.. SPDX-License-Identifier: GPL-2.0
.. c:namespace:: V4L

.. _v4l2-meta-fmt-qcom-isp-params:

**************************************
V4L2_META_FMT_QCOM_ISP_PARAMS ('QCIP')
**************************************

Configuration Parameters
========================

The ``V4L2_META_FMT_QCOM_ISP_PARAMS`` format carries image processing
configuration for the ISP engines found in the Qualcomm Camera Subsystem
(CAMSS). It is passed to a metadata output video node using the
:c:type:`v4l2_meta_format` interface.

Rather than a single struct containing sub-structs for each configurable area
of the ISP, parameters use the :ref:`v4l2-isp` parameters system, through which
groups of parameters are defined as distinct structs or "blocks" which may be
added to the data member of :c:type:`v4l2_isp_params_buffer`. Userspace is
responsible for populating the data member with the blocks that need to be
configured by the driver. Each block-specific struct embeds
:c:type:`v4l2_isp_params_block_header` as its first member and userspace must
populate the type member with a value from :c:type:`camss_params_block_type`.
Populated blocks must be placed consecutively in the data member, and the
combined size of all populated blocks must be set in the data_size member of
:c:type:`v4l2_isp_params_buffer`.

The set of supported blocks depends on the CAMSS engine consuming the buffer.
Currently the Offline Processing Engine (OPE) is the only engine defining
parameter blocks, exposed through its ``ope_params`` metadata output video
node; additional engines and blocks may be added to this format in the future.

Blocks whose header does not carry V4L2_ISP_PARAMS_FL_BLOCK_ENABLE leave the
corresponding hardware module bypassed. Blocks omitted from a buffer keep
their previously programmed configuration.

The following example populates an OPE parameters buffer with a white balance
and a gamma correction block:

.. code-block:: c

	struct v4l2_isp_params_buffer *params =
		(struct v4l2_isp_params_buffer *)buffer;

	params->version = V4L2_ISP_PARAMS_VERSION_V1;
	params->data_size = 0;

	void *data = (void *)params->data;

	struct camss_params_ope_wb_gain *wb =
		(struct camss_params_ope_wb_gain *)data;

	wb->header.type = CAMSS_PARAMS_OPE_WB_GAIN;
	wb->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	wb->header.size = sizeof(struct camss_params_ope_wb_gain);

	/* Unity gain on all three channels (15uQ10, 1024 = 1.0) */
	wb->g_gain = 1024;
	wb->b_gain = 1024;
	wb->r_gain = 1024;

	data += sizeof(struct camss_params_ope_wb_gain);
	params->data_size += sizeof(struct camss_params_ope_wb_gain);

	struct camss_params_ope_gamma *gamma =
		(struct camss_params_ope_gamma *)data;

	gamma->header.type = CAMSS_PARAMS_OPE_GAMMA;
	gamma->header.flags |= V4L2_ISP_PARAMS_FL_BLOCK_ENABLE;
	gamma->header.size = sizeof(struct camss_params_ope_gamma);

	/* Identity curve (pass-through, gamma 1.0) */
	for (unsigned int i = 0; i < CAMSS_OPE_GAMMA_LUT_SIZE; i++)
		gamma->glut[i] = gamma->blut[i] = gamma->rlut[i] = 257 * i;

	data += sizeof(struct camss_params_ope_gamma);
	params->data_size += sizeof(struct camss_params_ope_gamma);

Qualcomm CAMSS ISP uAPI data types
==================================

.. kernel-doc:: include/uapi/linux/qcom-camss-config.h
