/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021-2022 NXP
 */

#ifndef ELE_BASE_MSG_H
#define ELE_BASE_MSG_H

#define MAX_RECV_SIZE 31
#define MAX_RECV_SIZE_BYTES (MAX_RECV_SIZE * sizeof(u32))
#define MAX_MESSAGE_SIZE 31
#define MAX_MESSAGE_SIZE_BYTES (MAX_MESSAGE_SIZE * sizeof(u32))

#define MESSAGING_VERSION_6		0x6

#define ELE_OEM_CNTN_AUTH_REQ	0x87
#define ELE_VERIFY_IMAGE_REQ		0x88
#define ELE_RELEASE_CONTAINER_REQ	0x89
#define ELE_READ_FUSE_REQ		0x97
#define OTP_UNIQ_ID			0x01
#define OTFAD_CONFIG			0x2

#define ELE_VERSION			0x6
#define ELE_SUCCESS_IND		0xD6
#define ELE_FAILURE_IND		0x29

#define ELE_MSG_DATA_NUM		10

#define ELE_OEM_CNTN_AUTH_REQ_SIZE	3
#define ELE_VERIFY_IMAGE_REQ_SIZE	2
#define ELE_REL_CONTAINER_REQ_SIZE	1


int read_common_fuse(uint16_t fuse_index, u32 *value);

#endif
