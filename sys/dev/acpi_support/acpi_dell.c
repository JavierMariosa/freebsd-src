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
 * Driver for Dell laptop extras that are reached through the Dell SMBIOS
 * calling interface over ACPI-WMI.
 *
 * The firmware exposes settings as "tokens".  The list of tokens (ID,
 * location and value) is published in the SMBIOS table as structures of
 * type 0xDA, and tokens are read and written by passing a request buffer to
 * a WMI method.  Currently only the battery charge settings are supported.
 */

#include <sys/cdefs.h>
#include "opt_acpi.h"
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>
#include <dev/smbios/smbios.h>
#include "acpi_wmi_if.h"

#define _COMPONENT	ACPI_OEM
ACPI_MODULE_NAME("Dell")
ACPI_SERIAL_DECL(dell, "Dell SMBIOS WMI device");

static MALLOC_DEFINE(M_ACPIDELL, "acpi_dell", "Dell SMBIOS WMI data");

#define ACPI_DELL_SMBIOS_GUID	"A80593CE-A997-11DA-B012-B622A1EF5492"
#define ACPI_DELL_DESC_GUID	"8D9DDCBC-A997-11DA-B012-B622A1EF5492"

/* WMI descriptor: "DELL WMI", interface version, buffer size, hotfix. */
#define DELL_DESC_SIGNATURE	"DELL WMI"
#define DELL_DESC_MIN_LEN	16
#define DELL_DESC_SIZE_OFFSET	12
#define DELL_BUF_DEFAULT_SIZE	32768
#define DELL_BUF_MAX_SIZE	(1024 * 1024)

/* SMBIOS structure with the token table. */
#define DELL_SMBIOS_TYPE_TOKENS	0xda
#define DELL_SMBIOS_TYPE_END	127
#define DELL_TOKEN_END		0xffff

/* Calling interface classes and selectors. */
#define DELL_CLASS_TOKEN_READ	0
#define DELL_CLASS_TOKEN_WRITE	1
#define DELL_SELECT_TOKEN_STD	0

/* Battery charge tokens. */
#define DELL_TOKEN_BAT_PRIMARILY_AC	0x0341
#define DELL_TOKEN_BAT_ADAPTIVE		0x0342
#define DELL_TOKEN_BAT_CUSTOM		0x0343
#define DELL_TOKEN_BAT_STANDARD		0x0346
#define DELL_TOKEN_BAT_EXPRESS		0x0347
#define DELL_TOKEN_BAT_CUSTOM_START	0x0349
#define DELL_TOKEN_BAT_CUSTOM_END	0x034a

#define DELL_CHARGE_START_MIN	50
#define DELL_CHARGE_START_MAX	95
#define DELL_CHARGE_END_MIN	55
#define DELL_CHARGE_END_MAX	100
#define DELL_CHARGE_MIN_DIFF	5

struct dell_smbios_token {
	uint16_t	id;
	uint16_t	location;
	uint16_t	value;
} __packed;

struct dell_smbios_token_table {
	struct smbios_structure_header	header;
	uint16_t			cmd_io_address;
	uint8_t				cmd_io_code;
	uint32_t			supported_cmds;
	struct dell_smbios_token	tokens[];
} __packed;

struct dell_smbios_buffer {
	uint16_t	cmd_class;
	uint16_t	cmd_select;
	uint32_t	input[4];
	uint32_t	output[4];
} __packed;

struct dell_wmi_buffer {
	uint64_t			length;
	struct dell_smbios_buffer	std;
	uint32_t			argattrib;
	uint32_t			blength;
	uint8_t				data[];
} __packed;

struct acpi_dell_softc {
	device_t			dev;
	device_t			wmi_dev;

	struct dell_smbios_token	*tokens;
	int				ntokens;
	bool				tokens_done;

	struct dell_wmi_buffer		*buf;
	uint32_t			buf_size;
};

static const struct {
	const char	*name;
	uint16_t	token;
} acpi_dell_charge_modes[] = {
	{ "standard",		DELL_TOKEN_BAT_STANDARD },
	{ "express",		DELL_TOKEN_BAT_EXPRESS },
	{ "primarily_ac",	DELL_TOKEN_BAT_PRIMARILY_AC },
	{ "adaptive",		DELL_TOKEN_BAT_ADAPTIVE },
	{ "custom",		DELL_TOKEN_BAT_CUSTOM },
};

static struct dell_smbios_token *
acpi_dell_find_token(struct acpi_dell_softc *sc, uint16_t id)
{
	int i;

	for (i = 0; i < sc->ntokens; i++) {
		if (sc->tokens[i].id == id)
			return (&sc->tokens[i]);
	}
	return (NULL);
}

/*
 * SMBIOS walker callback.  It is run twice: first to count the tokens and
 * then, once sc->tokens has been allocated, to copy them.
 */
static void
acpi_dell_smbios_cb(struct smbios_structure_header *h, void *arg)
{
	struct acpi_dell_softc *sc = arg;
	struct dell_smbios_token_table *table;
	int i, n;

	if (sc->tokens_done)
		return;
	if (h->type == DELL_SMBIOS_TYPE_END) {
		sc->tokens_done = true;
		return;
	}
	if (h->type != DELL_SMBIOS_TYPE_TOKENS ||
	    h->length < sizeof(*table))
		return;

	table = (struct dell_smbios_token_table *)h;
	n = (h->length - sizeof(*table)) / sizeof(table->tokens[0]);
	for (i = 0; i < n; i++) {
		if (table->tokens[i].id == DELL_TOKEN_END)
			break;
		if (sc->tokens != NULL)
			sc->tokens[sc->ntokens] = table->tokens[i];
		sc->ntokens++;
	}
}

/*
 * Copy the token list out of the SMBIOS table.  The entry point is located
 * through the smbios(4) device, which has already found and validated it.
 */
static int
acpi_dell_read_tokens(struct acpi_dell_softc *sc)
{
	device_t smbios_dev;
	struct smbios_eps *eps;
	struct smbios3_eps *eps3;
	vm_paddr_t eps_pa, table_pa;
	vm_size_t eps_len, table_len;
	void *eps_va, *table;
	int entries, ntokens;

	smbios_dev = devclass_get_device(devclass_find("smbios"), 0);
	if (smbios_dev == NULL) {
		device_printf(sc->dev, "smbios(4) device not found\n");
		return (ENXIO);
	}
	eps_pa = bus_get_resource_start(smbios_dev, SYS_RES_MEMORY, 0);
	eps_len = bus_get_resource_count(smbios_dev, SYS_RES_MEMORY, 0);
	if (eps_pa == 0 || eps_len < sizeof(*eps3))
		return (ENXIO);

	eps_va = pmap_mapbios(eps_pa, eps_len);
	if (eps_va == NULL)
		return (ENOMEM);
	if (memcmp(eps_va, SMBIOS3_SIG, SMBIOS3_LEN) == 0) {
		eps3 = eps_va;
		table_pa = eps3->structure_table_address;
		table_len = eps3->structure_table_max_size;
		/* The 64-bit entry point has no count; stop at type 127. */
		entries = INT_MAX;
	} else if (eps_len >= sizeof(*eps)) {
		eps = eps_va;
		table_pa = eps->structure_table_address;
		table_len = eps->structure_table_length;
		entries = eps->number_structures;
	} else {
		table_pa = 0;
		table_len = 0;
		entries = 0;
	}
	pmap_unmapbios(eps_va, eps_len);
	if (table_pa == 0 || table_len == 0)
		return (ENXIO);

	table = pmap_mapbios(table_pa, table_len);
	if (table == NULL)
		return (ENOMEM);

	sc->tokens = NULL;
	sc->ntokens = 0;
	sc->tokens_done = false;
	smbios_walk_table(table, entries, table_len, acpi_dell_smbios_cb, sc);
	ntokens = sc->ntokens;
	if (ntokens > 0) {
		sc->tokens = malloc(ntokens * sizeof(sc->tokens[0]),
		    M_ACPIDELL, M_WAITOK | M_ZERO);
		sc->ntokens = 0;
		sc->tokens_done = false;
		smbios_walk_table(table, entries, table_len,
		    acpi_dell_smbios_cb, sc);
	}
	pmap_unmapbios(table, table_len);

	return (ntokens > 0 ? 0 : ENXIO);
}

/*
 * Ask the WMI descriptor for the size of the request buffer that the
 * firmware expects.
 */
static uint32_t
acpi_dell_get_buf_size(struct acpi_dell_softc *sc)
{
	ACPI_BUFFER out = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_OBJECT *obj;
	uint32_t size;

	size = DELL_BUF_DEFAULT_SIZE;
	if (!ACPI_WMI_PROVIDES_GUID_STRING(sc->wmi_dev, ACPI_DELL_DESC_GUID))
		return (size);
	if (ACPI_FAILURE(ACPI_WMI_GET_BLOCK(sc->wmi_dev, ACPI_DELL_DESC_GUID,
	    0, &out)))
		return (size);

	obj = out.Pointer;
	if (obj != NULL && obj->Type == ACPI_TYPE_BUFFER &&
	    obj->Buffer.Length >= DELL_DESC_MIN_LEN &&
	    memcmp(obj->Buffer.Pointer, DELL_DESC_SIGNATURE,
	    strlen(DELL_DESC_SIGNATURE)) == 0) {
		memcpy(&size, obj->Buffer.Pointer + DELL_DESC_SIZE_OFFSET,
		    sizeof(size));
		if (size < sizeof(struct dell_wmi_buffer) ||
		    size > DELL_BUF_MAX_SIZE)
			size = DELL_BUF_DEFAULT_SIZE;
	}
	if (out.Pointer != NULL)
		AcpiOsFree(out.Pointer);

	return (size);
}

/*
 * Run one SMBIOS call.  On success the firmware's reply is left in
 * sc->buf->std.
 */
static int
acpi_dell_call(struct acpi_dell_softc *sc, uint16_t cmd_class,
    uint16_t cmd_select, uint32_t in0, uint32_t in1)
{
	ACPI_BUFFER in, out = { ACPI_ALLOCATE_BUFFER, NULL };
	ACPI_OBJECT *obj;
	ACPI_STATUS status;
	int error;

	ACPI_SERIAL_ASSERT(dell);

	memset(sc->buf, 0, sc->buf_size);
	sc->buf->length = sc->buf_size - sizeof(sc->buf->length);
	sc->buf->std.cmd_class = cmd_class;
	sc->buf->std.cmd_select = cmd_select;
	sc->buf->std.input[0] = in0;
	sc->buf->std.input[1] = in1;

	in.Length = sc->buf_size;
	in.Pointer = sc->buf;
	status = ACPI_WMI_EVALUATE_CALL(sc->wmi_dev, ACPI_DELL_SMBIOS_GUID,
	    0, 1, &in, &out);
	if (ACPI_FAILURE(status)) {
		device_printf(sc->dev, "SMBIOS call %u/%u failed: %s\n",
		    cmd_class, cmd_select, AcpiFormatException(status));
		return (EIO);
	}

	error = EIO;
	obj = out.Pointer;
	if (obj != NULL && obj->Type == ACPI_TYPE_BUFFER &&
	    obj->Buffer.Length >= offsetof(struct dell_wmi_buffer, argattrib)) {
		memcpy(sc->buf, obj->Buffer.Pointer,
		    offsetof(struct dell_wmi_buffer, argattrib));
		switch ((int32_t)sc->buf->std.output[0]) {
		case 0:
			error = 0;
			break;
		case -2:
			error = EINVAL;
			break;
		default:
			error = EIO;
			break;
		}
	}
	if (out.Pointer != NULL)
		AcpiOsFree(out.Pointer);

	return (error);
}

static int
acpi_dell_token_read(struct acpi_dell_softc *sc, uint16_t id, uint32_t *val)
{
	struct dell_smbios_token *token;
	int error;

	token = acpi_dell_find_token(sc, id);
	if (token == NULL)
		return (ENODEV);
	error = acpi_dell_call(sc, DELL_CLASS_TOKEN_READ,
	    DELL_SELECT_TOKEN_STD, token->location, 0);
	if (error == 0)
		*val = sc->buf->std.output[1];
	return (error);
}

static int
acpi_dell_token_write(struct acpi_dell_softc *sc, uint16_t id, uint32_t val)
{
	struct dell_smbios_token *token;

	token = acpi_dell_find_token(sc, id);
	if (token == NULL)
		return (ENODEV);
	return (acpi_dell_call(sc, DELL_CLASS_TOKEN_WRITE,
	    DELL_SELECT_TOKEN_STD, token->location, val));
}

/*
 * A charge mode is selected by writing the value of its token to the
 * token's location; reading the location back tells which mode is active.
 */
static int
acpi_dell_charge_mode_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct acpi_dell_softc *sc = oidp->oid_arg1;
	struct dell_smbios_token *token;
	char mode[16];
	uint32_t val;
	int error, i;

	strlcpy(mode, "unknown", sizeof(mode));

	ACPI_SERIAL_BEGIN(dell);
	for (i = 0; i < nitems(acpi_dell_charge_modes); i++) {
		token = acpi_dell_find_token(sc,
		    acpi_dell_charge_modes[i].token);
		if (token == NULL)
			continue;
		error = acpi_dell_token_read(sc, token->id, &val);
		if (error == 0 && val == token->value) {
			strlcpy(mode, acpi_dell_charge_modes[i].name,
			    sizeof(mode));
			break;
		}
	}

	error = sysctl_handle_string(oidp, mode, sizeof(mode), req);
	if (error != 0 || req->newptr == NULL)
		goto out;

	error = EINVAL;
	for (i = 0; i < nitems(acpi_dell_charge_modes); i++) {
		if (strcmp(mode, acpi_dell_charge_modes[i].name) != 0)
			continue;
		token = acpi_dell_find_token(sc,
		    acpi_dell_charge_modes[i].token);
		if (token == NULL)
			error = ENODEV;
		else
			error = acpi_dell_token_write(sc, token->id,
			    token->value);
		break;
	}
out:
	ACPI_SERIAL_END(dell);
	return (error);
}

/*
 * Custom charge thresholds.  oid_arg2 is the token of the threshold being
 * accessed.  The firmware only honours them in the "custom" charge mode.
 */
static int
acpi_dell_charge_threshold_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct acpi_dell_softc *sc = oidp->oid_arg1;
	uint16_t id = oidp->oid_arg2;
	uint32_t start, end;
	int arg, error;

	ACPI_SERIAL_BEGIN(dell);
	error = acpi_dell_token_read(sc, DELL_TOKEN_BAT_CUSTOM_START, &start);
	if (error == 0)
		error = acpi_dell_token_read(sc, DELL_TOKEN_BAT_CUSTOM_END,
		    &end);
	if (error != 0)
		goto out;

	arg = (id == DELL_TOKEN_BAT_CUSTOM_START) ? start : end;
	error = sysctl_handle_int(oidp, &arg, 0, req);
	if (error != 0 || req->newptr == NULL)
		goto out;

	if (id == DELL_TOKEN_BAT_CUSTOM_START) {
		if (arg < DELL_CHARGE_START_MIN ||
		    arg > DELL_CHARGE_START_MAX ||
		    arg + DELL_CHARGE_MIN_DIFF > (int)end) {
			error = EINVAL;
			goto out;
		}
	} else {
		if (arg < DELL_CHARGE_END_MIN ||
		    arg > DELL_CHARGE_END_MAX ||
		    arg < (int)start + DELL_CHARGE_MIN_DIFF) {
			error = EINVAL;
			goto out;
		}
	}
	error = acpi_dell_token_write(sc, id, arg);
out:
	ACPI_SERIAL_END(dell);
	return (error);
}

static void
acpi_dell_identify(driver_t *driver, device_t parent)
{
	/* Don't do anything if driver is disabled. */
	if (acpi_disabled("dell"))
		return;

	/* Add only a single device instance. */
	if (device_find_child(parent, "acpi_dell", DEVICE_UNIT_ANY) != NULL)
		return;

	/* Check the SMBIOS call GUID to see whether system is compatible. */
	if (!ACPI_WMI_PROVIDES_GUID_STRING(parent, ACPI_DELL_SMBIOS_GUID))
		return;

	if (BUS_ADD_CHILD(parent, 0, "acpi_dell", DEVICE_UNIT_ANY) == NULL)
		device_printf(parent, "add acpi_dell child failed\n");
}

static int
acpi_dell_probe(device_t dev)
{
	if (!ACPI_WMI_PROVIDES_GUID_STRING(device_get_parent(dev),
	    ACPI_DELL_SMBIOS_GUID))
		return (EINVAL);
	device_set_desc(dev, "Dell SMBIOS WMI device");
	return (0);
}

static int
acpi_dell_attach(device_t dev)
{
	struct acpi_dell_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *children;
	int error;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->wmi_dev = device_get_parent(dev);

	error = acpi_dell_read_tokens(sc);
	if (error != 0) {
		device_printf(dev, "no SMBIOS tokens found\n");
		return (error);
	}
	sc->buf_size = acpi_dell_get_buf_size(sc);
	sc->buf = malloc(sc->buf_size, M_ACPIDELL, M_WAITOK | M_ZERO);
	if (bootverbose)
		device_printf(dev, "%d tokens, %u byte request buffer\n",
		    sc->ntokens, sc->buf_size);

	ctx = device_get_sysctl_ctx(dev);
	children = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));

	if (acpi_dell_find_token(sc, DELL_TOKEN_BAT_CUSTOM) != NULL ||
	    acpi_dell_find_token(sc, DELL_TOKEN_BAT_STANDARD) != NULL) {
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO, "charge_mode",
		    CTLTYPE_STRING | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
		    acpi_dell_charge_mode_sysctl, "A",
		    "Battery charge mode: standard, express, primarily_ac, "
		    "adaptive or custom");
	}
	if (acpi_dell_find_token(sc, DELL_TOKEN_BAT_CUSTOM_START) != NULL &&
	    acpi_dell_find_token(sc, DELL_TOKEN_BAT_CUSTOM_END) != NULL) {
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO,
		    "charge_start_threshold",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc,
		    DELL_TOKEN_BAT_CUSTOM_START,
		    acpi_dell_charge_threshold_sysctl, "I",
		    "Battery level (percent) below which charging starts "
		    "in custom charge mode");
		SYSCTL_ADD_PROC(ctx, children, OID_AUTO,
		    "charge_stop_threshold",
		    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc,
		    DELL_TOKEN_BAT_CUSTOM_END,
		    acpi_dell_charge_threshold_sysctl, "I",
		    "Battery level (percent) at which charging stops "
		    "in custom charge mode");
	}

	return (0);
}

static int
acpi_dell_detach(device_t dev)
{
	struct acpi_dell_softc *sc;

	sc = device_get_softc(dev);
	free(sc->buf, M_ACPIDELL);
	free(sc->tokens, M_ACPIDELL);

	return (0);
}

static device_method_t acpi_dell_methods[] = {
	DEVMETHOD(device_identify, acpi_dell_identify),
	DEVMETHOD(device_probe, acpi_dell_probe),
	DEVMETHOD(device_attach, acpi_dell_attach),
	DEVMETHOD(device_detach, acpi_dell_detach),

	DEVMETHOD_END
};

static driver_t	acpi_dell_driver = {
	"acpi_dell",
	acpi_dell_methods,
	sizeof(struct acpi_dell_softc),
};

DRIVER_MODULE(acpi_dell, acpi_wmi, acpi_dell_driver, 0, 0);
MODULE_DEPEND(acpi_dell, acpi_wmi, 1, 1, 1);
MODULE_DEPEND(acpi_dell, acpi, 1, 1, 1);
