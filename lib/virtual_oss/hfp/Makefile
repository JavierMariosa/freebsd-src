.PATH:		${SRCTOP}/usr.bin/bluetooth/rfcomm_sppd

SHLIB_NAME= 	voss_hfp.so

SRCS=		hfp.c \
		rfcomm_sdp.c

LIBADD=		bluetooth sdp pthread

# rfcomm_sdp.c is shared with rfcomm_sppd(1), which builds at this level.
WARNS?=		2

.include <bsd.lib.mk>
