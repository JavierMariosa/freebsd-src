/*
 * ng_ubt_qca.c
 */

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
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/types.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/taskqueue.h>

#include "usbdevs.h"
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#define	USB_DEBUG_VAR usb_debug
#include <dev/usb/usb_debug.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/ng_parse.h>
#include <netgraph/bluetooth/include/ng_bluetooth.h>
#include <netgraph/bluetooth/include/ng_hci.h>
#include <netgraph/bluetooth/include/ng_ubt.h>
#include <netgraph/bluetooth/drivers/ubt/ng_ubt_var.h>

static device_probe_t	ubt_qca_probe;

/*
 * Qualcomm Atheros QCA61x4 ("ROME") Bluetooth devices.  They come up
 * running a ROM firmware and need a rampatch and an NVM configuration
 * downloaded by qcabtfw(8) before they can be used.
 */
const STRUCT_USB_HOST_ID ubt_qca_devs[] =
{
	{ USB_VPI(0x0cf3, 0x535b, 0) },
	{ USB_VPI(0x0cf3, 0xe007, 0) },
	{ USB_VPI(0x0cf3, 0xe009, 0) },
	{ USB_VPI(0x0cf3, 0xe010, 0) },
	{ USB_VPI(0x0cf3, 0xe300, 0) },
	{ USB_VPI(0x0cf3, 0xe301, 0) },
	{ USB_VPI(0x0cf3, 0xe360, 0) },
	{ USB_VPI(0x0cf3, 0xe500, 0) },
};
const size_t ubt_qca_devs_sizeof = sizeof(ubt_qca_devs);

/* Vendor request reporting which firmware parts have been downloaded. */
#define	UBT_QCA_CHECK_STATUS	0x05
#define	UBT_QCA_SYSCFG_UPDATED	0x40
#define	UBT_QCA_PATCH_UPDATED	0x80
#define	UBT_QCA_TIMEOUT		2000	/* ms */

/*
 * Attach only once both the rampatch and the NVM have been downloaded.
 * Until then the device is left to qcabtfw(8), which resets it when done
 * so that it is probed again.
 */
static int
ubt_qca_probe(device_t dev)
{
	struct usb_attach_arg	*uaa = device_get_ivars(dev);
	struct usb_device_request req;
	uint8_t status;
	int error;

	if (uaa->usb_mode != USB_MODE_HOST)
		return (ENXIO);

	if (uaa->info.bIfaceIndex != 0)
		return (ENXIO);

	error = usbd_lookup_id_by_uaa(ubt_qca_devs, sizeof(ubt_qca_devs), uaa);
	if (error != 0)
		return (error);

	memset(&req, 0, sizeof(req));
	req.bmRequestType = UT_READ_VENDOR_DEVICE;
	req.bRequest = UBT_QCA_CHECK_STATUS;
	USETW(req.wLength, sizeof(status));
	status = 0;
	if (usbd_do_request_flags(uaa->device, NULL, &req, &status, 0, NULL,
	    UBT_QCA_TIMEOUT) != USB_ERR_NORMAL_COMPLETION)
		return (ENXIO);

	DPRINTFN(2, "status 0x%02x\n", status);

	if ((status & UBT_QCA_PATCH_UPDATED) == 0 ||
	    (status & UBT_QCA_SYSCFG_UPDATED) == 0)
		return (ENXIO);

	return (BUS_PROBE_DEFAULT);
}

/*
 * Module interface. Attach and detach methods, netgraph node type
 * registration and PNP string are inherited from ng_ubt.c driver.
 */

static device_method_t	ubt_qca_methods[] =
{
	DEVMETHOD(device_probe,	ubt_qca_probe),
	DEVMETHOD_END
};

DEFINE_CLASS_1(ubt, ubt_qca_driver, ubt_qca_methods,
    sizeof(struct ubt_softc), ubt_driver);
DRIVER_MODULE(ng_ubt_qca, uhub, ubt_qca_driver, 0, 0);
MODULE_VERSION(ng_ubt_qca, NG_BLUETOOTH_VERSION);
MODULE_DEPEND(ng_ubt_qca, netgraph, NG_ABI_VERSION, NG_ABI_VERSION,
    NG_ABI_VERSION);
MODULE_DEPEND(ng_ubt_qca, ng_hci, NG_BLUETOOTH_VERSION, NG_BLUETOOTH_VERSION,
    NG_BLUETOOTH_VERSION);
MODULE_DEPEND(ng_ubt_qca, usb, 1, 1, 1);
