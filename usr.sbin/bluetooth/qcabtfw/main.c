/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Javier Mariosa
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Firmware loader for Qualcomm Atheros QCA61x4 ("ROME") USB Bluetooth
 * adaptors.  The device boots from ROM and needs two files downloaded
 * before it is usable: a rampatch and an NVM configuration.  Once both
 * are in place the device is reset so that ng_ubt(4) attaches to it.
 */

#include <sys/param.h>
#include <sys/stat.h>
#include <sys/endian.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <libusb.h>

#define	QCABT_DEFAULT_FIRMWARE_PATH	"/usr/share/firmware/qcabt"
#define	QCABT_STAMP_FMT			"/var/run/qcabtfw.%u.%u"
#define	QCABT_STAMP_WINDOW		30	/* seconds */

/* Vendor requests */
#define	QCABT_DFU_DOWNLOAD		0x01
#define	QCABT_CHECK_STATUS		0x05
#define	QCABT_GET_TARGET_VERSION	0x09

/* QCABT_CHECK_STATUS bits */
#define	QCABT_SYSCFG_UPDATED		0x40
#define	QCABT_PATCH_UPDATED		0x80

#define	QCABT_FLAG_MULTI_NVM		0x80

#define	QCABT_BULK_EP			0x02
#define	QCABT_DFU_PACKET_LEN		4096
#define	QCABT_CTRL_TIMEOUT		5000	/* ms */
#define	QCABT_DFU_TIMEOUT		3000	/* ms */

#define	QCABT_MAX_FW_SIZE		(4 * 1024 * 1024)

struct qcabt_version {
	uint32_t	rom_version;
	uint32_t	patch_version;
	uint32_t	ram_version;
	uint8_t		chip_id;
	uint8_t		platform_id;
	uint16_t	flag;
	uint8_t		reserved[4];
} __packed;

struct qcabt_rampatch_version {
	uint16_t	rom_version_high;
	uint16_t	rom_version_low;
	uint16_t	patch_version;
} __packed;

struct qcabt_device_info {
	uint32_t	rom_version;
	uint8_t		rampatch_hdr;	/* length of header in rampatch */
	uint8_t		nvm_hdr;	/* length of header in NVM */
	uint8_t		ver_offset;	/* offset of version in rampatch */
};

static const struct qcabt_device_info qcabt_devices[] = {
	{ 0x00000100, 20, 4,  8 },	/* Rome 1.0 */
	{ 0x00000101, 20, 4,  8 },	/* Rome 1.1 */
	{ 0x00000200, 28, 4, 16 },	/* Rome 2.0 */
	{ 0x00000201, 28, 4, 16 },	/* Rome 2.1 */
	{ 0x00000300, 28, 4, 16 },	/* Rome 3.0 */
	{ 0x00000302, 28, 4, 16 },	/* Rome 3.2 */
};

struct qcabt_devid {
	uint16_t	vendor_id;
	uint16_t	product_id;
};

static const struct qcabt_devid qcabt_list[] = {
	{ 0x0cf3, 0x535b },
	{ 0x0cf3, 0xe007 },
	{ 0x0cf3, 0xe009 },
	{ 0x0cf3, 0xe010 },
	{ 0x0cf3, 0xe300 },
	{ 0x0cf3, 0xe301 },
	{ 0x0cf3, 0xe360 },
	{ 0x0cf3, 0xe500 },
};

struct qcabt_firmware {
	char		*path;
	uint8_t		*buf;
	size_t		len;
};

static int	qcabt_do_debug = 0;
static int	qcabt_do_info = 0;

#define	qcabt_err(fmt, ...)						\
	fprintf(stderr, "qcabtfw: %s: "fmt"\n", __func__, ##__VA_ARGS__)
#define	qcabt_info(fmt, ...)	do {					\
	if (qcabt_do_info)						\
		fprintf(stderr, "%s: "fmt"\n", __func__, ##__VA_ARGS__);\
} while (0)
#define	qcabt_debug(fmt, ...)	do {					\
	if (qcabt_do_debug)						\
		fprintf(stderr, "%s: "fmt"\n", __func__, ##__VA_ARGS__);\
} while (0)

static libusb_device *
qcabt_find_device(libusb_context *ctx, int bus_id, int dev_id)
{
	libusb_device **list, *dev, *found = NULL;
	struct libusb_device_descriptor d;
	ssize_t cnt, i;
	size_t j;

	cnt = libusb_get_device_list(ctx, &list);
	if (cnt < 0) {
		qcabt_err("libusb_get_device_list() failed: code %lld",
		    (long long int)cnt);
		return (NULL);
	}

	for (i = 0; i < cnt && found == NULL; i++) {
		dev = list[i];
		if (bus_id != libusb_get_bus_number(dev) ||
		    dev_id != libusb_get_device_address(dev))
			continue;
		if (libusb_get_device_descriptor(dev, &d) != 0)
			break;
		for (j = 0; j < nitems(qcabt_list); j++) {
			if (d.idVendor == qcabt_list[j].vendor_id &&
			    d.idProduct == qcabt_list[j].product_id) {
				/* Take a reference so it's not freed. */
				found = libusb_ref_device(dev);
				break;
			}
		}
	}

	libusb_free_device_list(list, 1);
	return (found);
}

static int
qcabt_vendor_req(libusb_device_handle *hdl, uint8_t request, void *data,
    uint16_t size)
{
	int r;

	r = libusb_control_transfer(hdl,
	    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
	    request, 0, 0, data, size, QCABT_CTRL_TIMEOUT);
	if (r < 0) {
		qcabt_err("vendor request 0x%02x failed: %s", request,
		    libusb_strerror(r));
		return (-1);
	}
	if (r != size) {
		qcabt_err("vendor request 0x%02x: short read (%d of %u)",
		    request, r, size);
		return (-1);
	}
	return (0);
}

static int
qcabt_get_version(libusb_device_handle *hdl, struct qcabt_version *ver)
{
	if (qcabt_vendor_req(hdl, QCABT_GET_TARGET_VERSION, ver,
	    sizeof(*ver)) != 0)
		return (-1);
	qcabt_info("rom 0x%08x patch 0x%08x ram 0x%08x flag 0x%04x",
	    le32toh(ver->rom_version), le32toh(ver->patch_version),
	    le32toh(ver->ram_version), le16toh(ver->flag));
	return (0);
}

static const struct qcabt_device_info *
qcabt_get_info(uint32_t rom_version)
{
	size_t i;

	for (i = 0; i < nitems(qcabt_devices); i++) {
		if (qcabt_devices[i].rom_version == rom_version)
			return (&qcabt_devices[i]);
	}
	return (NULL);
}

static void
qcabt_fw_free(struct qcabt_firmware *fw)
{
	free(fw->path);
	free(fw->buf);
	memset(fw, 0, sizeof(*fw));
}

static int
qcabt_fw_read(struct qcabt_firmware *fw, const char *dir, const char *prefix,
    uint32_t rom_version)
{
	struct stat sb;
	ssize_t r;
	int fd;

	memset(fw, 0, sizeof(*fw));
	if (asprintf(&fw->path, "%s/%s_usb_%08x.bin", dir, prefix,
	    rom_version) < 0) {
		fw->path = NULL;
		return (-1);
	}

	fd = open(fw->path, O_RDONLY);
	if (fd < 0) {
		qcabt_err("open(%s): %s", fw->path, strerror(errno));
		goto fail;
	}
	if (fstat(fd, &sb) != 0 || sb.st_size <= 0 ||
	    sb.st_size > QCABT_MAX_FW_SIZE) {
		qcabt_err("%s: bad file size", fw->path);
		close(fd);
		goto fail;
	}
	fw->len = sb.st_size;
	fw->buf = malloc(fw->len);
	if (fw->buf == NULL) {
		close(fd);
		goto fail;
	}
	r = read(fd, fw->buf, fw->len);
	close(fd);
	if (r != (ssize_t)fw->len) {
		qcabt_err("%s: short read", fw->path);
		goto fail;
	}
	return (0);
fail:
	qcabt_fw_free(fw);
	return (-1);
}

/*
 * The first hdr_size bytes go to the control endpoint, the rest is sent
 * in chunks to the bulk endpoint.
 */
static int
qcabt_download(libusb_device_handle *hdl, const struct qcabt_firmware *fw,
    size_t hdr_size)
{
	size_t count, sent, size;
	int r, transferred;

	count = fw->len;
	size = MIN(count, hdr_size);
	r = libusb_control_transfer(hdl,
	    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
	    QCABT_DFU_DOWNLOAD, 0, 0, fw->buf, size, QCABT_CTRL_TIMEOUT);
	if (r < 0 || (size_t)r != size) {
		qcabt_err("%s: failed to send header: %s", fw->path,
		    r < 0 ? libusb_strerror(r) : "short write");
		return (-1);
	}
	sent = size;
	count -= size;

	/* The bulk endpoint needs time to switch from ACL to DFU. */
	usleep(20000);

	while (count > 0) {
		size = MIN(count, QCABT_DFU_PACKET_LEN);
		r = libusb_bulk_transfer(hdl, QCABT_BULK_EP, fw->buf + sent,
		    size, &transferred, QCABT_DFU_TIMEOUT);
		if (r != 0 || (size_t)transferred != size) {
			qcabt_err("%s: failed to send body at %zu: %s",
			    fw->path, sent,
			    r != 0 ? libusb_strerror(r) : "short write");
			return (-1);
		}
		sent += size;
		count -= size;
	}
	qcabt_debug("%s: %zu bytes sent", fw->path, sent);
	return (0);
}

/*
 * The rampatch has to be for this ROM and newer than what is running.
 */
static int
qcabt_check_rampatch(const struct qcabt_firmware *fw,
    const struct qcabt_device_info *info, const struct qcabt_version *ver)
{
	struct qcabt_rampatch_version rver;
	uint32_t rver_rom, ver_rom, ver_patch;
	uint16_t rver_patch;

	if (fw->len < info->ver_offset + sizeof(rver) ||
	    fw->len < info->rampatch_hdr) {
		qcabt_err("%s: file too short", fw->path);
		return (-1);
	}
	memcpy(&rver, fw->buf + info->ver_offset, sizeof(rver));
	ver_rom = le32toh(ver->rom_version);
	ver_patch = le32toh(ver->patch_version);
	rver_rom = le16toh(rver.rom_version_low);
	rver_patch = le16toh(rver.patch_version);
	if ((ver_rom & ~0xffffU) != 0)
		rver_rom |= (uint32_t)le16toh(rver.rom_version_high) << 16;

	qcabt_info("rampatch: rom 0x%x build 0x%x, device: rom 0x%x "
	    "build 0x%x", rver_rom, rver_patch, ver_rom, ver_patch);

	if (rver_rom != ver_rom || rver_patch <= ver_patch) {
		qcabt_err("%s: version does not match the device", fw->path);
		return (-1);
	}
	return (0);
}

/*
 * The firmware is expected to survive the reset that hands the device
 * over to the kernel.  Should it not, devd(8) would run us again and
 * again; the stamp file is used to detect that and to give up instead.
 */
static bool
qcabt_stamp_recent(const char *stamp)
{
	struct stat sb;

	if (stat(stamp, &sb) != 0)
		return (false);
	return (time(NULL) - sb.st_mtime < QCABT_STAMP_WINDOW);
}

static void
qcabt_stamp_touch(const char *stamp)
{
	int fd;

	fd = open(stamp, O_WRONLY | O_CREAT | O_TRUNC, 0600);
	if (fd >= 0) {
		(void)futimens(fd, NULL);
		close(fd);
	}
}

/*
 * Parse ugen name and extract device's bus and address
 */
static int
parse_ugen_name(char const *ugen, uint8_t *bus, uint8_t *addr)
{
	char *ep;

	if (strncmp(ugen, "ugen", 4) != 0)
		return (-1);

	*bus = (uint8_t)strtoul(ugen + 4, &ep, 10);
	if (*ep != '.')
		return (-1);

	*addr = (uint8_t)strtoul(ep + 1, &ep, 10);
	if (*ep != '\0')
		return (-1);

	return (0);
}

static void
usage(void)
{
	fprintf(stderr,
	    "Usage: qcabtfw (-D) -d ugenX.Y (-f firmware path) (-I)\n");
	fprintf(stderr, "    -D: enable debugging\n");
	fprintf(stderr, "    -d: device to operate upon\n");
	fprintf(stderr, "    -f: firmware path, if not default\n");
	fprintf(stderr, "    -I: enable informational output\n");
	exit(127);
}

int
main(int argc, char *argv[])
{
	libusb_context *ctx = NULL;
	libusb_device *dev = NULL;
	libusb_device_handle *hdl = NULL;
	const struct qcabt_device_info *info;
	struct qcabt_version ver;
	struct qcabt_firmware fw;
	const char *firmware_dir = QCABT_DEFAULT_FIRMWARE_PATH;
	char stamp[64];
	uint8_t bus_id = 0, dev_id = 0, status;
	bool claimed = false, devid_set = false, downloaded = false;
	int n, r, retcode = 1;

	while ((n = getopt(argc, argv, "Dd:f:hI")) != -1) {
		switch (n) {
		case 'd': /* ugen device name */
			devid_set = true;
			if (parse_ugen_name(optarg, &bus_id, &dev_id) < 0)
				usage();
			break;
		case 'D':
			qcabt_do_debug = 1;
			break;
		case 'f': /* firmware dir */
			firmware_dir = optarg;
			break;
		case 'I':
			qcabt_do_info = 1;
			break;
		case 'h':
		default:
			usage();
			/* NOTREACHED */
		}
	}
	if (!devid_set)
		usage();

	snprintf(stamp, sizeof(stamp), QCABT_STAMP_FMT, bus_id, dev_id);

	r = libusb_init(&ctx);
	if (r != 0) {
		qcabt_err("libusb_init failed: code %d", r);
		exit(127);
	}

	qcabt_debug("opening dev %d.%d", (int)bus_id, (int)dev_id);

	dev = qcabt_find_device(ctx, bus_id, dev_id);
	if (dev == NULL) {
		qcabt_err("device not found");
		goto shutdown;
	}

	r = libusb_open(dev, &hdl);
	if (r != 0) {
		qcabt_err("libusb_open() failed: code %d", r);
		goto shutdown;
	}

	/* Check if ng_ubt is attached */
	r = libusb_kernel_driver_active(hdl, 0);
	if (r < 0) {
		qcabt_err("libusb_kernel_driver_active() failed: code %d", r);
		goto shutdown;
	}
	if (r > 0) {
		qcabt_info("Firmware has already been downloaded");
		retcode = 0;
		goto shutdown;
	}

	if (qcabt_get_version(hdl, &ver) != 0)
		goto shutdown;
	info = qcabt_get_info(le32toh(ver.rom_version));
	if (info == NULL) {
		qcabt_err("unsupported ROM version 0x%08x",
		    le32toh(ver.rom_version));
		goto shutdown;
	}
	if (((le16toh(ver.flag) >> 8) & 0xff) == QCABT_FLAG_MULTI_NVM) {
		qcabt_err("devices with board specific NVM are not supported");
		goto shutdown;
	}

	if (qcabt_vendor_req(hdl, QCABT_CHECK_STATUS, &status,
	    sizeof(status)) != 0)
		goto shutdown;
	qcabt_info("status 0x%02x", status);

	if ((status & QCABT_PATCH_UPDATED) != 0 &&
	    (status & QCABT_SYSCFG_UPDATED) != 0) {
		/* Nothing to do; ng_ubt attaches once it is loaded. */
		qcabt_info("Firmware has already been downloaded");
		retcode = 0;
		goto shutdown;
	}

	if (qcabt_stamp_recent(stamp)) {
		qcabt_err("firmware was lost after the last download, "
		    "giving up");
		goto shutdown;
	}

	r = libusb_claim_interface(hdl, 0);
	if (r != 0) {
		qcabt_err("libusb_claim_interface() failed: %s",
		    libusb_strerror(r));
		goto shutdown;
	}
	claimed = true;

	if ((status & QCABT_PATCH_UPDATED) == 0) {
		if (qcabt_fw_read(&fw, firmware_dir, "rampatch",
		    le32toh(ver.rom_version)) != 0)
			goto shutdown;
		qcabt_info("loading rampatch %s", fw.path);
		r = qcabt_check_rampatch(&fw, info, &ver);
		if (r == 0)
			r = qcabt_download(hdl, &fw, info->rampatch_hdr);
		qcabt_fw_free(&fw);
		if (r != 0)
			goto shutdown;
		downloaded = true;

		if (qcabt_get_version(hdl, &ver) != 0)
			goto shutdown;
	}

	if ((status & QCABT_SYSCFG_UPDATED) == 0) {
		if (qcabt_fw_read(&fw, firmware_dir, "nvm",
		    le32toh(ver.rom_version)) != 0)
			goto shutdown;
		qcabt_info("loading NVM %s", fw.path);
		r = qcabt_download(hdl, &fw, info->nvm_hdr);
		qcabt_fw_free(&fw);
		if (r != 0)
			goto shutdown;
		downloaded = true;
	}

	if (qcabt_vendor_req(hdl, QCABT_CHECK_STATUS, &status,
	    sizeof(status)) == 0)
		qcabt_info("status after download 0x%02x", status);

	retcode = 0;

shutdown:
	if (claimed)
		(void)libusb_release_interface(hdl, 0);

	if (retcode == 0 && downloaded) {
		qcabt_stamp_touch(stamp);

		/* Ask kernel driver to probe and attach device again */
		r = libusb_reset_device(hdl);
		if (r != 0)
			qcabt_err("libusb_reset_device() failed: %s",
			    libusb_strerror(r));
	}

	if (hdl != NULL)
		libusb_close(hdl);

	if (dev != NULL)
		libusb_unref_device(dev);

	if (ctx != NULL)
		libusb_exit(ctx);

	if (retcode == 0)
		qcabt_info("Firmware download is successful!");
	else
		qcabt_err("Firmware download failed!");

	return (retcode);
}
