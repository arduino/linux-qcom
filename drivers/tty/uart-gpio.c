/*
 * SPDX-License-Identifier: GPL-2.0-only
 * Ported from https://github.com/thinkty/software-serial
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/kobject.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define START_BIT         (0)
#define STOP_BIT          (1)

#define DEVICE_NAME       "soft_serial"
#define DEVICE_WORKQUEUE  "soft_serial-workqueue"
#define DEVICE_FIFO_SIZE  (64)  /* Size of the circular buffer */
#define DEVICE_TIMEOUT    (100) /* Time (ms) to wait until buffer has space */

/**
 * drvdata - device data TODO: clean up this stuff
 *
 * @param tx_fifo Circular buffer for storing byte(s) to transmit
 * @param tx_fifo_mutex Mutex for the transmit buffer
 * @param writer_waitq Queue for writers when tx buffer is full
 *
 * @param rx_fifo Circular buffer for storing byte(s) received
 * @param rx_fifo_mutex Mutex for the receival buffer
 * @param reader_waitq Queue for readers when rx buffer is empty
 *
 * @param closing Flag to stop work from self-queueing to workqueue
 * @param miscdev Used for getting drvdata using container_of
 * @param rx_irq IRQ number for getting the interrupts of RX line
 * @param rx_gpio GPIO descriptor for receival
 * @param tx_gpio GPIO descriptor for transmission
 * @param tx_timer High resolution timer to handle reading in a byte
 * @param tx_timer High resolution timer to handle bit-banging
 * @param delay Nanoseconds before sending the next "bit" (computed by baudrate)
 */
struct drvdata {
    struct kfifo tx_fifo;
    struct mutex tx_fifo_mutex;
    wait_queue_head_t writer_waitq;

    struct kfifo rx_fifo;
    struct mutex rx_fifo_mutex;
    wait_queue_head_t reader_waitq;

    bool closing;
    struct miscdevice miscdev;
    int rx_irq;
    struct gpio_desc * rx_gpio;
    struct gpio_desc * tx_gpio;
    struct hrtimer rx_timer;
    struct hrtimer tx_timer;
    ktime_t delay;
};

static int baudrate = 9600; /* Lower the baudrate, more reliable */
module_param(baudrate, int, 0);
MODULE_PARM_DESC(baudrate, "\tBaudrate of the device (default=9600)");

/**
 * Function prototypes for file operations
 */
static int release(struct inode * inode, struct file * filp);
static ssize_t write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos);
static ssize_t read(struct file * filp, char __user * buf, size_t count, loff_t * f_pos);
static irqreturn_t rx_isr(int irq, void * data);

/**
 * File operations given as function pointers. .open is handled by default and
 * sets file->private_data to point to the structure.
 */
static struct file_operations fops = {
    .owner = THIS_MODULE,
    .release = release,
    .read = read,
    .write = write,
};

/**
 * @brief Get the driver data from the given file.
 *
 * @param filp Pointer to the open file for the device
 *
 * @returns Device driver data
 */
static inline struct drvdata * get_drvdata(struct file * filp)
{
    struct miscdevice * dev = filp->private_data;
    return container_of(dev, struct drvdata, miscdev);
}

/**
 * @brief Release is called when the device file descriptor is closed. However,
 * deallocation of device related data structures is done on exit module.
 *
 * @param inode Pointer to the file containing device metadata
 * @param filp Pointer to the open file for the device
 *
 * @returns 0
 */
static int release(struct inode * inode, struct file * filp)
{
	unsigned int mn = iminor(inode);
    pr_info(DEVICE_NAME ": released %s%u\n", DEVICE_NAME, mn);
    return 0;
}

/**
 * @brief Write to the device buffer (fifo). Since there can be simultaneous
 * writes, a mutex lock is used. If the buffer is full, the write operation
 * blocks until a wake up is received. If the user specified non-blocking, it
 * will return as soon as it is blocked.
 *
 * @param filp Pointer to the open file for the device
 * @param buf User-space buffer
 * @param count Number of bytes to write
 * @param f_pos Not used
 *
 * @returns Number of bytes successfully written or a negative errno value.
 */
static ssize_t write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos)
{
    char tbuf[DEVICE_FIFO_SIZE] = {0};
    size_t to_copy = 0;
    int err = 0;
    struct drvdata * dd = get_drvdata(filp);

    /* At maximum, the size of buffer */
    to_copy = count < DEVICE_FIFO_SIZE ? count : DEVICE_FIFO_SIZE;
    to_copy -= copy_from_user(tbuf, buf, to_copy);

    /* By being interruptible, when given any signal, the process will just
    give up on acquiring the lock and return -EINTR. */
    err = mutex_lock_interruptible(&dd->tx_fifo_mutex);
    if (err < 0) {
        return err;
    }

    /* Check in a loop since another writer may get to kfifo first. This could
    potentially be a thundering herd problem */
    while (kfifo_is_full(&dd->tx_fifo) && !dd->closing) {
        mutex_unlock(&dd->tx_fifo_mutex);

        /* If non-blocking, return immediately */
        if (filp->f_flags & O_NDELAY || filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }

        /* Sleep until space available, closing device, interrupt, or timeout */
        err = wait_event_interruptible_hrtimeout(dd->writer_waitq, dd->closing || !kfifo_is_full(&dd->tx_fifo), ms_to_ktime(DEVICE_TIMEOUT));
        if (err < 0) {
            return err;
        }
        if (dd->closing) {
            return -ECANCELED;
        }

        err = mutex_lock_interruptible(&dd->tx_fifo_mutex);
        if (err < 0) {
            return err;
        }
    }

    /* If closing device, return */
    if (dd->closing) {
        mutex_unlock(&dd->tx_fifo_mutex);
        return -ECANCELED;
    }

    /* Depending on the buffer vacancy, it might write less than specified */
    to_copy = kfifo_in(&dd->tx_fifo, tbuf, to_copy);
    mutex_unlock(&dd->tx_fifo_mutex);

    /* If the clock isn't waiting or executing handler, start the timer. */
    if (!hrtimer_active(&dd->tx_timer)) {
        hrtimer_start(&dd->tx_timer, dd->delay, HRTIMER_MODE_REL);
    }

    return to_copy;
}

/**
 * @brief Callback function called by hrtimer to handle bit-banging. Since this
 * function runs in interrupt context, do not sleep or trigger a resched. The
 * timer is stopped after transmitting the last bit. wake_up() and kfifo_get()
 * are safe to call in interrupt context. The function transmits until there are
 * no bytes left in the fifo.
 *
 * @param timer The hrtimer that called this callback function
 *
 * @returns HRTIMER_RESTART until all the bits (start bit, data (8), and stop
 * bit (1)) are sent.
 */
static enum hrtimer_restart tx_timer_callback(struct hrtimer * timer)
{
    static unsigned int bit = -1;
    static unsigned char byte = 0;
    struct drvdata * dd = container_of(timer, struct drvdata, tx_timer);

    /* Start bit */
    if (bit == -1) {
        /* If nothing was retrieved, wake up the writer_waitq for more. */
        /* Since there is only 1 actual device to write to, no locks here */
        if (kfifo_get(&dd->tx_fifo, &byte) == 0) {
            wake_up_interruptible(&dd->writer_waitq);
            return HRTIMER_NORESTART;
        }

        gpiod_set_value(dd->tx_gpio, START_BIT);
        bit++;
    }
    /* Data bits (8 bits) */
    else if (bit < 8) {
        gpiod_set_value(dd->tx_gpio, (byte & (1 << bit)) >> bit);
        bit++;
    }
    /* Stop bit */
    else {
        gpiod_set_value(dd->tx_gpio, STOP_BIT);
        bit = -1;
        byte = 0;
    }

    /* Restart timer to handle next bit */
    hrtimer_forward_now(timer, dd->delay);
    return HRTIMER_RESTART;
}

/**
 * @brief Read from the device buffer (fifo). Since it doesn't make sense to
 * have multiple readers on a single device, the function assumes there is only
 * one reader. If the buffer is empty, the read operation blocks until a wake up
 * is received. If the user specified non-blocking, it will return as soon as it
 * is blocked.
 *
 * @param filp Pointer to the open file for the device
 * @param buf User-space buffer
 * @param count Number of bytes to read
 * @param f_pos Not used
 *
 * @returns Number of bytes successfully read or a negative errno value.
 */
static ssize_t read(struct file * filp, char __user * buf, size_t count, loff_t * f_pos)
{
    int to_copy = count < DEVICE_FIFO_SIZE ? count : DEVICE_FIFO_SIZE;
    int err = 0;
    struct drvdata * dd = get_drvdata(filp);

    if (kfifo_is_empty(&dd->rx_fifo)) {

        /* If non-blocking, return immediately */
        if (filp->f_flags & O_NDELAY || filp->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }

        /* Sleep until new byte(s) available, closing device, interrupt */
        err = wait_event_interruptible(dd->reader_waitq, dd->closing || !kfifo_is_empty(&dd->rx_fifo));
        if (err < 0) {
            return err;
        }
        if (dd->closing) {
            return -ECANCELED;
        }
    }

    /* If closing device, return */
    if (dd->closing) {
        return -ECANCELED;
    }

    /* Depending on the buffer vacancy, it might write less than specified */
    err = kfifo_to_user(&dd->rx_fifo, buf, to_copy, &to_copy);
    if (err < 0) {
        pr_err(DEVICE_NAME ": kfifo_to_user() failed, bad address\n");
        return err;
    }

    return to_copy;
}

/**
 * @brief Interrupt service routine for the RX line interrupts. As set in the
 * device tree overlay, the interrupt handler is triggered on the falling edge
 * since the line is usually 1 (stop bit) and falls to 0 (start bit) at the
 * beginning of data transmission. Once the interrupt is triggered, the handler
 * checks on the rx_timer which has an interval of the baudrate (same as the
 * tx_timer). If the rx_timer is active (waiting or executing handler), it means
 * that it is processing a byte already.
 *
 * @param irq IRQ number specified by the device
 * @param data Pointer to the platform device
 *
 * @returns IRQ_HANDLED.
 */
static irqreturn_t rx_isr(int irq, void * data)
{
    struct platform_device * pdev = (struct platform_device *) data;
    struct drvdata * dd = (struct drvdata *) dev_get_drvdata(&pdev->dev);

    /* If the rx_timer is not running, start it to read in a new byte */
    if (!hrtimer_active(&dd->rx_timer)) {
        hrtimer_start(&dd->rx_timer, dd->delay, HRTIMER_MODE_REL);
    }

    return IRQ_HANDLED;
}

/**
 * @brief Callback function called by hrtimer to handle reading in a byte. The
 * function gpiod_get_value is safe to call in a non-sleep context.
 *
 * @param timer The hrtimer that called this callback function
 *
 * @returns HRTIMER_RESTART until all the bits (start bit, data (8), and stop
 * bit (1)) are read.
 */
static enum hrtimer_restart rx_timer_callback(struct hrtimer * timer)
{
    static unsigned int bit = 0;
    static unsigned char byte = 0;
    unsigned char temp;
    struct drvdata * dd = container_of(timer, struct drvdata, rx_timer);

    /* Data bits (8 bits). LSB come first */
    if (bit < 8) {
        byte |= gpiod_get_value(dd->rx_gpio) << bit;
        bit++;
    } else {

        /* If the fifo is full, pull last one out to make room */
        if (kfifo_is_full(&dd->rx_fifo) && kfifo_get(&dd->rx_fifo, &temp)) {
            pr_warn(DEVICE_NAME ": rx_timer_callback() dropped a byte to make room for new bytes\n");
        }

        if (kfifo_put(&dd->rx_fifo, byte) == 0) {
            pr_err(DEVICE_NAME ": kfifo_put() says fifo is full although it shouldn't be\n");
        }
        bit = 0;
        byte = 0;

        /* Wake up any readers waiting */
        wake_up_interruptible(&dd->reader_waitq);
        return HRTIMER_NORESTART;
    }

    /* Restart timer to handle next bit */
    hrtimer_forward_now(timer, dd->delay);
    return HRTIMER_RESTART;
}

/**
 * @brief Initialize the device by allocating its numbers (major, minor),
 * create a misc (single character) device, and initialize the driver data.
 *
 * @param pdev Platform device
 *
 * @returns 0 on success, less than 0 on error.
 */
static int dt_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct drvdata * dd = NULL;

    /* Allocate driver (zero-ed) data from kernel memory */
    dd = devm_kzalloc(&pdev->dev, sizeof(struct drvdata), GFP_KERNEL);
    if (!dd) {
        pr_err(DEVICE_NAME ": kzalloc() failed\n");
        return -ENOMEM;
    }
    dev_set_drvdata(&pdev->dev, dd);

    /* Allocate tx & rx fifos */
    ret = kfifo_alloc(&dd->rx_fifo, DEVICE_FIFO_SIZE, GFP_KERNEL);
    if (ret != 0) {
        pr_err(DEVICE_NAME ": kfifo_alloc(rx_fifo) failed\n");
        goto DT_PROBE_RX_KFIFO_ALLOC;
    }
    ret = kfifo_alloc(&dd->tx_fifo, DEVICE_FIFO_SIZE, GFP_KERNEL);
    if (ret != 0) {
        pr_err(DEVICE_NAME ": kfifo_alloc(tx_fifo) failed\n");
        goto DT_PROBE_TX_KFIFO_ALLOC;
    }

    /* Get the GPIO descriptors from the pin numbers */
    dd->rx_gpio = devm_gpiod_get_index(&pdev->dev, "serial", 1, GPIOD_IN);
    if (IS_ERR(dd->rx_gpio)) {
        ret = PTR_ERR(dd->rx_gpio);
        pr_err(DEVICE_NAME ": devm_gpiod_get(rx) failed\n");
        goto DT_PROBE_GPIOD_GET_RX;
    }
    dd->tx_gpio = devm_gpiod_get_index(&pdev->dev, "serial", 0, GPIOD_OUT_HIGH);
    if (IS_ERR(dd->tx_gpio)) {
        ret = PTR_ERR(dd->tx_gpio);
        pr_err(DEVICE_NAME ": devm_gpiod_get(tx) failed\n");
        goto DT_PROBE_GPIOD_GET_TX;
    }
    gpiod_set_value(dd->tx_gpio, 1); /* Set as stop bit */

    /* Initialize various wait queues and fifo mutexes */
    init_waitqueue_head(&dd->reader_waitq);
    mutex_init(&dd->rx_fifo_mutex);
    init_waitqueue_head(&dd->writer_waitq);
    mutex_init(&dd->tx_fifo_mutex);

    /* Initialize the high resolution timer for RX and TX */
    dd->delay = ktime_set(0, NSEC_PER_SEC / baudrate);
    hrtimer_setup(&dd->rx_timer, rx_timer_callback, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    hrtimer_setup(&dd->tx_timer, tx_timer_callback, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

    /* Get IRQ from platform device (device tree) and request IRQ */
    dd->rx_irq = platform_get_irq(pdev, 0);
    if (dd->rx_irq < 0) {
        ret = -ENODATA;
        pr_err(DEVICE_NAME ": platform_get_irq() failed\n");
        goto DT_PROBE_REQUEST_IRQ;
    }
    ret = devm_request_irq(&pdev->dev, dd->rx_irq, rx_isr, IRQF_TRIGGER_NONE, dev_name(&pdev->dev), pdev);
    if (ret < 0) {
        pr_err(DEVICE_NAME ": devm_request_irq() failed\n");
        goto DT_PROBE_REQUEST_IRQ;
    }

    /* Make the device available to the kernel & user */
    dd->miscdev.minor = MISC_DYNAMIC_MINOR;
    dd->miscdev.name = DEVICE_NAME;
    dd->miscdev.fops = &fops;
    dd->miscdev.mode = S_IRUGO | S_IWUGO;
    ret = misc_register(&dd->miscdev);
    if (ret < 0) {
        pr_err(DEVICE_NAME ": misc_register() failed\n");
        goto DT_PROBE_MISC_REG;
    }

    pr_info(DEVICE_NAME ": successful init with baudrate=%d\n", baudrate);
    return 0;

DT_PROBE_MISC_REG:
DT_PROBE_REQUEST_IRQ:
    dd->closing = true;
    wake_up_interruptible_all(&dd->reader_waitq);
    wake_up_interruptible_all(&dd->writer_waitq);
    hrtimer_cancel(&dd->rx_timer);
    hrtimer_cancel(&dd->tx_timer);
    mutex_destroy(&dd->rx_fifo_mutex);
    mutex_destroy(&dd->tx_fifo_mutex);
DT_PROBE_GPIOD_GET_TX:
DT_PROBE_GPIOD_GET_RX:
    kfifo_free(&dd->tx_fifo);
DT_PROBE_TX_KFIFO_ALLOC:
    kfifo_free(&dd->rx_fifo);
DT_PROBE_RX_KFIFO_ALLOC:
    return ret;
}

/**
 * @brief Free the device data, misc device, and the allocated device numbers.
 *
 * @param pdev Platform device
 */
static void dt_remove(struct platform_device *pdev)
{
    struct drvdata * dd = dev_get_drvdata(&pdev->dev);
    if (!dd) {
        pr_err(DEVICE_NAME ": driver data does not exist\n");
        return;
    }

    /* Unregister misc device to stop new incoming bytes */
    misc_deregister(&dd->miscdev);

    /* Mark as closing and stop readers, writers and tx hrtimer */
    dd->closing = true;
    wake_up_interruptible_all(&dd->reader_waitq);
    wake_up_interruptible_all(&dd->writer_waitq);
    hrtimer_cancel(&dd->rx_timer);
    hrtimer_cancel(&dd->tx_timer);

    /* Cleanup driver data. Since dd is allocated via devm_*, no need to free */
    mutex_destroy(&dd->rx_fifo_mutex);
    mutex_destroy(&dd->tx_fifo_mutex);
    kfifo_free(&dd->rx_fifo);
    kfifo_free(&dd->tx_fifo);

    pr_info(DEVICE_NAME ": exit\n");
}

/**
 * Specifying the name of the device in device tree using the overlay
 */
static struct of_device_id dt_ids[] = {
    { .compatible = "thinkty,soft_serial", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, dt_ids);

/**
 * N7D platform device operations given as function pointers
 */
static struct platform_driver soft_serial_platform_driver = {
    .probe = dt_probe,
    .remove = dt_remove,
    .driver = {
        .name = DEVICE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(dt_ids),
    },
};

/* Macro for module init and exit */
module_platform_driver(soft_serial_platform_driver);

MODULE_AUTHOR("Tae Yoon Kim");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Software serial device driver");
MODULE_VERSION("0:0.1");
