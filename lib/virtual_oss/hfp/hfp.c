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
 * Bluetooth headset backend with a microphone: Hands-Free Profile (HFP)
 * and Headset Profile (HSP), audio gateway role.
 *
 * Unlike A2DP, these profiles carry audio in both directions.  Control
 * goes over an RFCOMM channel using AT commands and audio over a SCO
 * connection as 8 kHz, 16-bit, mono PCM; the CVSD coding used on the air
 * is done by the Bluetooth controller.
 *
 * The recording and playback backends share one connection to the
 * headset.
 */

#include <sys/param.h>
#include <sys/socket.h>
#include <sys/soundcard.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#define	L2CAP_SOCKET_CHECKED
#include <bluetooth.h>
#include <sdp.h>

#include "backend.h"

#define	HFP_PREFIX		"/dev/bluetooth-hfp/"
#define	HFP_SAMPLE_RATE		8000
#define	HFP_SLC_TIMEOUT		5000	/* ms */
#define	HFP_SCO_PKT_DEFAULT	48	/* bytes */
#define	HFP_SCO_PKT_MAX		1024	/* bytes */

#define	DPRINTF(...) printf("backend_hfp: " __VA_ARGS__)

int	rfcomm_channel_lookup(bdaddr_t const *, bdaddr_t const *, int,
	    int *, int *);

struct hfp_state {
	pthread_mutex_t	mtx;
	int		refs;		/* open backends */
	bool		connected;
	bool		is_hfp;		/* HFP, as opposed to HSP */
	volatile bool	slc_done;	/* service level connection is up */
	volatile bool	dead;		/* headset went away */

	int		rfcomm_fd;
	int		sco_fd;
	pthread_t	at_thread;
	bool		at_thread_running;

	/* size of the SCO packets the headset sends, used for sending too */
	volatile int	pkt_size;
	int		mtu;

	/* partially consumed received packet */
	uint8_t		rx_buf[HFP_SCO_PKT_MAX];
	int		rx_off;
	int		rx_len;

	/* partially filled packet to send */
	uint8_t		tx_buf[HFP_SCO_PKT_MAX];
	int		tx_len;
};

static struct hfp_state hfp_state = {
	.mtx = PTHREAD_MUTEX_INITIALIZER,
	.rfcomm_fd = -1,
	.sco_fd = -1,
};

static int
hfp_at_send(struct hfp_state *st, const char *text)
{
	char buf[256];
	int len;

	len = snprintf(buf, sizeof(buf), "\r\n%s\r\n", text);
	if (len < 0 || len >= (int)sizeof(buf))
		return (-1);
	if (write(st->rfcomm_fd, buf, len) != len)
		return (-1);
	return (0);
}

/*
 * Answer one AT command from the headset.  The gateway advertises no
 * optional features, which keeps the headset on the mandatory CVSD codec
 * and makes the service level connection complete after AT+CMER.
 */
static int
hfp_at_handle(struct hfp_state *st, const char *cmd)
{
	int error = 0;

	if (strncmp(cmd, "AT+BRSF=", 8) == 0) {
		error = hfp_at_send(st, "+BRSF: 0");
	} else if (strcmp(cmd, "AT+CIND=?") == 0) {
		error = hfp_at_send(st, "+CIND: (\"service\",(0,1)),"
		    "(\"call\",(0,1)),(\"callsetup\",(0-3)),"
		    "(\"callheld\",(0-2)),(\"signal\",(0-5)),"
		    "(\"roam\",(0,1)),(\"battchg\",(0-5))");
	} else if (strcmp(cmd, "AT+CIND?") == 0) {
		error = hfp_at_send(st, "+CIND: 1,0,0,0,5,0,5");
	} else if (strncmp(cmd, "AT+CMER=", 8) == 0) {
		st->slc_done = true;
	} else if (strcmp(cmd, "AT+CHLD=?") == 0) {
		error = hfp_at_send(st, "+CHLD: (0,1,2,3)");
	} else if (strncmp(cmd, "AT+VGS=", 7) == 0 ||
	    strncmp(cmd, "AT+VGM=", 7) == 0 ||
	    strncmp(cmd, "AT+CKPD=", 8) == 0 ||
	    strncmp(cmd, "AT+CLIP=", 8) == 0 ||
	    strncmp(cmd, "AT+CCWA=", 8) == 0 ||
	    strncmp(cmd, "AT+CMEE=", 8) == 0 ||
	    strncmp(cmd, "AT+NREC=", 8) == 0 ||
	    strncmp(cmd, "AT+BIA=", 7) == 0) {
		/* Nothing to do but to acknowledge. */
	} else {
		return (hfp_at_send(st, "ERROR"));
	}
	if (error != 0)
		return (error);
	return (hfp_at_send(st, "OK"));
}

/*
 * Read from the RFCOMM channel and answer the commands found.  Returns
 * -1 once the channel is gone.
 */
static int
hfp_at_service(struct hfp_state *st, char *line, size_t *plen, size_t size)
{
	char buf[128];
	ssize_t n, i;

	n = read(st->rfcomm_fd, buf, sizeof(buf));
	if (n == 0 || (n < 0 && errno != EINTR && errno != EAGAIN))
		return (-1);
	for (i = 0; i < n; i++) {
		if (buf[i] == '\r' || buf[i] == '\n') {
			if (*plen > 0) {
				line[*plen] = '\0';
				*plen = 0;
				if (hfp_at_handle(st, line) != 0)
					return (-1);
			}
		} else if (*plen < size - 1) {
			line[(*plen)++] = buf[i];
		}
	}
	return (0);
}

static void *
hfp_at_thread(void *arg)
{
	struct hfp_state *st = arg;
	char line[128];
	size_t len = 0;

	while (hfp_at_service(st, line, &len, sizeof(line)) == 0)
		;
	st->dead = true;
	return (NULL);
}

static void
hfp_disconnect(struct hfp_state *st)
{
	if (st->sco_fd >= 0)
		shutdown(st->sco_fd, SHUT_RDWR);
	if (st->rfcomm_fd >= 0)
		shutdown(st->rfcomm_fd, SHUT_RDWR);
	if (st->at_thread_running) {
		pthread_join(st->at_thread, NULL);
		st->at_thread_running = false;
	}
	if (st->sco_fd >= 0) {
		close(st->sco_fd);
		st->sco_fd = -1;
	}
	if (st->rfcomm_fd >= 0) {
		close(st->rfcomm_fd);
		st->rfcomm_fd = -1;
	}
	st->connected = false;
}

static int
hfp_connect(struct hfp_state *st, const char *devname)
{
	struct sockaddr_rfcomm ra;
	struct sockaddr_sco sa;
	struct pollfd pfd;
	bdaddr_t raddr;
	char line[128];
	size_t len = 0;
	socklen_t optlen;
	int channel, error, mtu;

	if (strstr(devname, HFP_PREFIX) != devname) {
		DPRINTF("Invalid device name '%s'\n", devname);
		return (-1);
	}
	devname += sizeof(HFP_PREFIX) - 1;

	if (!bt_aton(devname, &raddr)) {
		struct hostent *he;

		if ((he = bt_gethostbyname(devname)) == NULL) {
			DPRINTF("Could not get host by name\n");
			return (-1);
		}
		bdaddr_copy(&raddr, (bdaddr_t *)he->h_addr);
	}

	st->dead = false;
	st->slc_done = false;
	st->pkt_size = HFP_SCO_PKT_DEFAULT;
	st->rx_off = st->rx_len = st->tx_len = 0;

	/* Prefer HFP and fall back to the simpler HSP. */
	st->is_hfp = true;
	if (rfcomm_channel_lookup(NULL, &raddr, SDP_SERVICE_CLASS_HANDSFREE,
	    &channel, &error) != 0) {
		st->is_hfp = false;
		if (rfcomm_channel_lookup(NULL, &raddr,
		    SDP_SERVICE_CLASS_HEADSET, &channel, &error) != 0) {
			DPRINTF("Headset offers neither HFP nor HSP: %s\n",
			    strerror(error));
			return (-1);
		}
	}
	DPRINTF("Using %s on RFCOMM channel %d\n",
	    st->is_hfp ? "HFP" : "HSP", channel);

	st->rfcomm_fd = socket(PF_BLUETOOTH, SOCK_STREAM,
	    BLUETOOTH_PROTO_RFCOMM);
	if (st->rfcomm_fd < 0) {
		DPRINTF("Could not create RFCOMM socket: %s\n",
		    strerror(errno));
		goto fail;
	}
	memset(&ra, 0, sizeof(ra));
	ra.rfcomm_len = sizeof(ra);
	ra.rfcomm_family = AF_BLUETOOTH;
	if (bind(st->rfcomm_fd, (struct sockaddr *)&ra, sizeof(ra)) < 0) {
		DPRINTF("Could not bind RFCOMM socket: %s\n", strerror(errno));
		goto fail;
	}
	bdaddr_copy(&ra.rfcomm_bdaddr, &raddr);
	ra.rfcomm_channel = channel;
	if (connect(st->rfcomm_fd, (struct sockaddr *)&ra, sizeof(ra)) < 0) {
		DPRINTF("Could not connect RFCOMM socket: %s\n",
		    strerror(errno));
		goto fail;
	}

	/*
	 * A hands-free unit starts talking as soon as the channel is up and
	 * audio may only be opened after that negotiation is done.
	 */
	if (st->is_hfp) {
		pfd.fd = st->rfcomm_fd;
		pfd.events = POLLIN;
		while (!st->slc_done) {
			if (poll(&pfd, 1, HFP_SLC_TIMEOUT) <= 0) {
				DPRINTF("Timeout setting up HFP\n");
				goto fail;
			}
			if (hfp_at_service(st, line, &len, sizeof(line)) != 0) {
				DPRINTF("Headset closed the connection\n");
				goto fail;
			}
		}
	}
	if (pthread_create(&st->at_thread, NULL, hfp_at_thread, st) != 0) {
		DPRINTF("Could not create thread\n");
		goto fail;
	}
	st->at_thread_running = true;

	st->sco_fd = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BLUETOOTH_PROTO_SCO);
	if (st->sco_fd < 0) {
		DPRINTF("Could not create SCO socket: %s\n", strerror(errno));
		goto fail;
	}
	memset(&sa, 0, sizeof(sa));
	sa.sco_len = sizeof(sa);
	sa.sco_family = AF_BLUETOOTH;
	if (bind(st->sco_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		DPRINTF("Could not bind SCO socket: %s\n", strerror(errno));
		goto fail;
	}
	bdaddr_copy(&sa.sco_bdaddr, &raddr);
	if (connect(st->sco_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
		DPRINTF("Could not connect SCO socket: %s\n", strerror(errno));
		goto fail;
	}
	mtu = 0;
	optlen = sizeof(mtu);
	if (getsockopt(st->sco_fd, SOL_SCO, SO_SCO_MTU, &mtu, &optlen) < 0 ||
	    mtu <= 0 || mtu > HFP_SCO_PKT_MAX)
		mtu = HFP_SCO_PKT_DEFAULT;
	st->mtu = mtu & ~1;
	if (st->pkt_size > st->mtu)
		st->pkt_size = st->mtu;
	DPRINTF("Audio connection is up, MTU %d\n", st->mtu);

	st->connected = true;
	return (0);
fail:
	hfp_disconnect(st);
	return (-1);
}

static int
hfp_open(struct voss_backend *pbe, const char *devname, int samplerate,
    int bufsize __unused, int *pchannels, int *pformat)
{
	struct hfp_state *st = pbe->arg;
	int error = 0;

	if (samplerate != HFP_SAMPLE_RATE) {
		DPRINTF("Sample rate must be %d Hz (-r %d), not %d Hz\n",
		    HFP_SAMPLE_RATE, HFP_SAMPLE_RATE, samplerate);
		return (-1);
	}
	if ((*pformat & AFMT_S16_LE) == 0) {
		DPRINTF("16-bit samples are required (-b 16)\n");
		return (-1);
	}

	pthread_mutex_lock(&st->mtx);
	if (st->connected && st->dead && st->refs == 0)
		hfp_disconnect(st);
	if (!st->connected)
		error = hfp_connect(st, devname);
	if (error == 0)
		st->refs++;
	pthread_mutex_unlock(&st->mtx);
	if (error != 0)
		return (-1);

	*pformat = AFMT_S16_LE;
	*pchannels = 1;
	return (0);
}

static void
hfp_close(struct voss_backend *pbe)
{
	struct hfp_state *st = pbe->arg;

	pthread_mutex_lock(&st->mtx);
	if (st->refs > 0 && --st->refs == 0)
		hfp_disconnect(st);
	pthread_mutex_unlock(&st->mtx);
}

/*
 * Reading blocks until the headset has sent enough packets, which paces
 * the caller at the headset's sample clock.
 */
static int
hfp_rec_transfer(struct voss_backend *pbe, void *ptr, int len)
{
	struct hfp_state *st = pbe->arg;
	uint8_t *dst = ptr;
	int delta, done = 0;
	ssize_t n;

	while (done < len) {
		if (st->dead)
			return (-1);
		if (st->rx_len == 0) {
			n = read(st->sco_fd, st->rx_buf, sizeof(st->rx_buf));
			if (n < 0 && errno == EINTR)
				continue;
			if (n <= 0)
				return (-1);
			st->rx_off = 0;
			st->rx_len = n & ~1;
			if (st->rx_len > 0 && st->rx_len <= st->mtu)
				st->pkt_size = st->rx_len;
			continue;
		}
		delta = MIN(len - done, st->rx_len);
		memcpy(dst + done, st->rx_buf + st->rx_off, delta);
		done += delta;
		st->rx_off += delta;
		st->rx_len -= delta;
	}
	return (done);
}

static int
hfp_play_transfer(struct voss_backend *pbe, void *ptr, int len)
{
	struct hfp_state *st = pbe->arg;
	const uint8_t *src = ptr;
	int delta, done = 0, pkt_size;

	if (st->dead)
		return (-1);

	while (done < len) {
		pkt_size = st->pkt_size;
		if (st->tx_len < pkt_size) {
			delta = MIN(len - done, pkt_size - st->tx_len);
			memcpy(st->tx_buf + st->tx_len, src + done, delta);
			st->tx_len += delta;
			done += delta;
		}
		if (st->tx_len >= pkt_size) {
			/*
			 * Dropping a packet when the socket is full is
			 * better than stalling the mixer.
			 */
			if (send(st->sco_fd, st->tx_buf, st->tx_len,
			    MSG_DONTWAIT) < 0 && errno != EAGAIN &&
			    errno != ENOBUFS && errno != EINTR)
				return (-1);
			st->tx_len = 0;
		}
	}
	return (done);
}

static void
hfp_delay(struct voss_backend *pbe __unused, int *pdelay)
{
	*pdelay = -1;
}

struct voss_backend voss_backend_hfp_rec = {
	.open = hfp_open,
	.close = hfp_close,
	.transfer = hfp_rec_transfer,
	.delay = hfp_delay,
	.arg = &hfp_state,
};

struct voss_backend voss_backend_hfp_play = {
	.open = hfp_open,
	.close = hfp_close,
	.transfer = hfp_play_transfer,
	.delay = hfp_delay,
	.arg = &hfp_state,
};
