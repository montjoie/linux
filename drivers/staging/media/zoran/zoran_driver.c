// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Zoran zr36057/zr36067 PCI controller driver, for the
 * Pinnacle/Miro DC10/DC10+/DC30/DC30+, Iomega Buz, Linux
 * Media Labs LML33/LML33R10.
 *
 * Copyright (C) 2000 Serguei Miridonov <mirsev@cicese.mx>
 *
 * Changes for BUZ by Wolfgang Scherr <scherr@net4you.net>
 *
 * Changes for DC10/DC30 by Laurent Pinchart <laurent.pinchart@skynet.be>
 *
 * Changes for LML33R10 by Maxim Yevtyushkin <max@linuxmedialabs.com>
 *
 * Changes for videodev2/v4l2 by Ronald Bultje <rbultje@ronald.bitfreak.net>
 *
 * Based on
 *
 * Miro DC10 driver
 * Copyright (C) 1999 Wolfgang Scherr <scherr@net4you.net>
 *
 * Iomega Buz driver version 1.0
 * Copyright (C) 1999 Rainer Johanni <Rainer@Johanni.de>
 *
 * buz.0.0.3
 * Copyright (C) 1998 Dave Perks <dperks@ibm.net>
 *
 * bttv - Bt848 frame grabber driver
 * Copyright (C) 1996,97,98 Ralph  Metzler (rjkm@thp.uni-koeln.de)
 *                        & Marcus Metzler (mocm@thp.uni-koeln.de)
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#include <linux/spinlock.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/videobuf-core.h>
#include "videocodec.h"

#include <asm/byteorder.h>
#include <linux/io.h>
#include <linux/uaccess.h>

#include <linux/mutex.h>
#include "zoran.h"
#include "zoran_device.h"
#include "zoran_card.h"

const struct zoran_format zoran_formats[] = {
	{
		.name = "15-bit RGB LE",
		.fourcc = V4L2_PIX_FMT_RGB555,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 15,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB555 | ZR36057_VFESPFR_ErrDif |
			   ZR36057_VFESPFR_LittleEndian,
	}, {
		.name = "15-bit RGB BE",
		.fourcc = V4L2_PIX_FMT_RGB555X,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 15,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB555 | ZR36057_VFESPFR_ErrDif,
	}, {
		.name = "16-bit RGB LE",
		.fourcc = V4L2_PIX_FMT_RGB565,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 16,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB565 | ZR36057_VFESPFR_ErrDif |
			   ZR36057_VFESPFR_LittleEndian,
	}, {
		.name = "16-bit RGB BE",
		.fourcc = V4L2_PIX_FMT_RGB565X,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 16,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB565 | ZR36057_VFESPFR_ErrDif,
	}, {
		.name = "24-bit RGB",
		.fourcc = V4L2_PIX_FMT_BGR24,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 24,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB888 | ZR36057_VFESPFR_Pack24,
	}, {
		.name = "32-bit RGB LE",
		.fourcc = V4L2_PIX_FMT_BGR32,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 32,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB888 | ZR36057_VFESPFR_LittleEndian,
	}, {
		.name = "32-bit RGB BE",
		.fourcc = V4L2_PIX_FMT_RGB32,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.depth = 32,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_RGB888,
	}, {
		.name = "4:2:2, packed, YUYV",
		.fourcc = V4L2_PIX_FMT_YUYV,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.depth = 16,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_YUV422,
	}, {
		.name = "4:2:2, packed, UYVY",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.depth = 16,
		.flags = ZORAN_FORMAT_CAPTURE,
		.vfespfr = ZR36057_VFESPFR_YUV422 | ZR36057_VFESPFR_LittleEndian,
	}, {
		.name = "Hardware-encoded Motion-JPEG",
		.fourcc = V4L2_PIX_FMT_MJPEG,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.depth = 0,
		.flags = ZORAN_FORMAT_CAPTURE |
#ifndef COMPLIANCE
			 ZORAN_FORMAT_PLAYBACK |
#endif
			 ZORAN_FORMAT_COMPRESSED,
	}
};

int zoran_status(struct zoran *zr)
{
	u32 v;
	int w, h, r, g, b;
	int still;
	int Still_LitEndian = 0;

	pci_info(zr->pci_dev, "Codec %s\n", codec_name(zr->codec_mode));
	pci_info(zr->pci_dev, "Norm is %s\n", norm_name(zr->norm));
	pci_info(zr->pci_dev, "Input is %d %s\n", zr->input, zr->card.input[zr->input].name);
	pci_info(zr->pci_dev, "MAPMODE is %s\n", map_mode(zr->map_mode));

	v = btread(ZR36057_VFEHCR);
	pci_info(zr->pci_dev, "ZR36057_VFEHCR %x\n", v);
	v = btread(ZR36057_VFEVCR);
	pci_info(zr->pci_dev, "ZR36057_VFEVCR %x\n", v);
	v = btread(ZR36057_VFESPFR);
	pci_info(zr->pci_dev, "ZR36057_VFESPFR %x\n", v);
	switch (v & 0x24) {
	case 0:
		pci_info(zr->pci_dev, "ZR36057_VFESPFR YUV422");
		break;
	default:
		pci_info(zr->pci_dev, "ZR36057_VFESPFR unk format");
	}

	v = btread(ZR36057_ICR);
	pci_info(zr->pci_dev, "ZR36057_ICR %x\n", v);
	if (v & ZR36057_ICR_CodRepIRQ)
		 pci_info(zr->pci_dev, "ZR36057_ICR ZR36057_ICR_CodRepIRQ\n");
	if (v & ZR36057_ICR_JPEGRepIRQ)
		 pci_info(zr->pci_dev, "ZR36057_ICR ZR36057_ICR_JPEGRepIRQ\n");
	if (v & ZR36057_ICR_IntPinEn)
		 pci_info(zr->pci_dev, "ZR36057_ICR ZR36057_ICR_IntPinEn\n");

	v = btread(ZR36057_VSSFGR);
	pci_info(zr->pci_dev, "ZR36057_VSSFGR %x\n", v);
	if (v & ZR36057_VSSFGR_SnapShot)
		 pci_info(zr->pci_dev, "ZR36057_VSSFGR ZR36057_VSSFGR_SnapShot\n");
	if (v & ZR36057_VSSFGR_FrameGrab)
		 pci_info(zr->pci_dev, "ZR36057_VSSFGR ZR36057_VSSFGR_FrameGrab\n");

	v = btread(ZR36057_VDCR);
	w = v & GENMASK(9, 0);
	h = (v >> ZR36057_VDCR_VidWinHt) & GENMASK(9, 0);
	pci_info(zr->pci_dev, "ZR36057_VDCR %x %dx%d\n", v, w, h);
	if (v & ZR36057_VDCR_VidEn)
		pci_info(zr->pci_dev, "ZR36057_VDCR Video Enabled\n");
	else
		pci_info(zr->pci_dev, "ZR36057_VDCR Video Disabled\n");
	if (!(v & ZR36057_VDCR_Triton)) {
		pci_info(zr->pci_dev, "ZR36057_VDCR TRITON\n");
	}
	w = (v & GENMASK(30, 23)) >> ZR36057_VDCR_MinPix;
	pci_info(zr->pci_dev, "ZR36057_VDCR Min pix %d\n", w);

	v = btread(ZR36057_MMTR);
	pci_info(zr->pci_dev, "ZR36057_MMTR %x\n", v);
	v = btread(ZR36057_MMBR);
	pci_info(zr->pci_dev, "ZR36057_MMBR %x\n", v);

	v = btread(ZR36057_OCR);
	if (v & ZR36057_OCR_OvlEnable)
		pci_info(zr->pci_dev, "ZR36057_OCR %x enabled\n", v);
	else
		pci_info(zr->pci_dev, "ZR36057_OCR %x disabled\n", v);

	v = btread(ZR36057_SPGPPCR);
	pci_info(zr->pci_dev, "ZR36057_SPGPPCR %x\n", v);

	v = btread(ZR36057_GPPGCR1);
	pci_info(zr->pci_dev, "ZR36057_GPPGCR1 %x\n", v);

	v = btread(ZR36057_MCSAR);
	pci_info(zr->pci_dev, "ZR36057_MCSAR %x\n", v);

	v = btread(ZR36057_MCTCR);
	pci_info(zr->pci_dev, "ZR36057_MCTCR %x\n", v);

	v = btread(ZR36057_MCMPR);
	pci_info(zr->pci_dev, "ZR36057_MCMPR %x\n", v);
/*
	v = btread(ZR36057_ISR);
	pci_info(zr->pci_dev, "ZR36057_ISR %x\n", v);

	v = btread(ZR36057_ICR);
	pci_info(zr->pci_dev, "ZR36057_ICR %x\n", v);
*//*
	v = btread(ZR36057_I2CBR);
	pci_info(zr->pci_dev, "ZR36057_I2CBR %x\n", v);
*/
	v = btread(ZR36057_JMC);
	pci_info(zr->pci_dev, "ZR36057_JMC %x\n", v);
	Still_LitEndian = v & 1;
	if (Still_LitEndian)
		pci_info(zr->pci_dev, "ZR36057_JMC Little Endian %x\n", v);
	else
		pci_info(zr->pci_dev, "ZR36057_JMC Big Endian %x\n", v);
	if (v & BIT(31))
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG Mode %x\n", v);
	else
		pci_info(zr->pci_dev, "ZR36057_JMC MPEG Mode%x \n", v);
	w = (v & GENMASK(30, 29)) >> 29;
	switch(w) {
	case 0:
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG: Still image Decompression %x\n", v);
		break;
	case 1:
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG: Still image Compression %x\n", v);
		break;
	case 2:
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG: Motion video Decompression %x\n", v);
		break;
	case 3:
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG: Motion video Compression %x\n", v);
		break;
	default:
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG: bad mode %x\n", v);
		break;
	}
	if (v & BIT(5))
		pci_info(zr->pci_dev, "ZR36057_JMC JPEG: GO %x\n", v);
		
	v = btread(ZR36057_JPC);
	pci_info(zr->pci_dev, "ZR36057_JPC %x\n", v);

	v = btread(ZR36057_VSP);
	w = (v & GENMASK(23,16)) >> 16;
	h = v & GENMASK(15,0);
	pci_info(zr->pci_dev, "ZR36057_VSP %x (%d %d)\n", v, w, h);

	v = btread(ZR36057_HSP);
	h = (v & GENMASK(31,16)) >> 16;
	w = v & GENMASK(15, 0);
	pci_info(zr->pci_dev, "ZR36057_HSP %x (%d %d)\n", v, h, w);

	v = btread(ZR36057_FHAP);
	pci_info(zr->pci_dev, "ZR36057_FHAP %x\n", v);

	v = btread(ZR36057_FVAP);
	pci_info(zr->pci_dev, "ZR36057_FVAP %x\n", v);

	v = btread(ZR36057_FPP);
	pci_info(zr->pci_dev, "ZR36057_FPP %x\n", v);

	v = btread(ZR36057_JCBA);
	pci_info(zr->pci_dev, "ZR36057_JCBA %x\n", v);
	v = btread(ZR36057_JCFT);
	pci_info(zr->pci_dev, "ZR36057_JCFT %x fifo=%ld\n", v, v & GENMASK(7, 0));

	v = btread(ZR36057_JCGI);
	pci_info(zr->pci_dev, "ZR36057_JCGI %x JPEGuestID=%lx JPEGuestReg=%lx\n", v,
		(v & GENMASK(6, 4)),
		(v & GENMASK(2, 0)));

	v = btread(ZR36057_STR);
	pci_info(zr->pci_dev, "ZR36057_STR %x\n", v);
	if (Still_LitEndian) {
		still = v & BIT(31);
		r = (v & GENMASK(23, 16)) >> 16;
		g = (v & GENMASK(18, 8)) >> 8;
		b = v & GENMASK(7, 0);
	} else {
		still = v & BIT(7);
		r = (v & GENMASK(15, 8)) >> 8;
		g = (v & GENMASK(23, 16)) >> 16;
		b = (v & GENMASK(31, 24)) >> 24;
	}
	if (still)
		pci_info(zr->pci_dev, "ZR36057_STR availlable R=%d G=%d B=%d\n", r, g, b);
	else
		pci_info(zr->pci_dev, "ZR36057_STR not availlable R=%d G=%d B=%d\n", r, g, b);
		
/*	for (i = 0; i < BUZ_NUM_STAT_COM; i++)
		pci_info(zr->pci_dev, "STAT COM %d %x buf=%x\n", i, zr->stat_com[i], zr->inuse[i]);*/
	return 0;
}


#define NUM_FORMATS ARRAY_SIZE(zoran_formats)

	/* small helper function for calculating buffersizes for v4l2
	 * we calculate the nearest higher power-of-two, which
	 * will be the recommended buffersize */
static __u32 zoran_v4l2_calc_bufsize(struct zoran_jpg_settings *settings)
{
	__u8 div = settings->VerDcm * settings->HorDcm * settings->TmpDcm;
	__u32 num = (1024 * 512) / (div);
	__u32 result = 2;

	num--;
	while (num) {
		num >>= 1;
		result <<= 1;
	}

	if (result > jpg_bufsize)
		return jpg_bufsize;
	if (result < 8192)
		return 8192;

	pr_info("%s %d\n", __func__, result);
	return result;
}

/*
 *   Allocate the MJPEG grab buffers.
 *
 *   If a Natoma chipset is present and this is a revision 1 zr36057,
 *   each MJPEG buffer needs to be physically contiguous.
 *   (RJ: This statement is from Dave Perks' original driver,
 *   I could never check it because I have a zr36067)
 *
 *   RJ: The contents grab buffers needs never be accessed in the driver.
 *       Therefore there is no need to allocate them with vmalloc in order
 *       to get a contiguous virtual memory space.
 *       I don't understand why many other drivers first allocate them with
 *       vmalloc (which uses internally also get_zeroed_page, but delivers you
 *       virtual addresses) and then again have to make a lot of efforts
 *       to get the physical address.
 *
 *   Ben Capper:
 *       On big-endian architectures (such as ppc) some extra steps
 *       are needed. When reading and writing to the stat_com array
 *       and fragment buffers, the device expects to see little-
 *       endian values. The use of cpu_to_le32() and le32_to_cpu()
 *       in this function (and one or two others in zoran_device.c)
 *       ensure that these values are always stored in little-endian
 *       form, regardless of architecture. The zr36057 does Very Bad
 *       Things on big endian architectures if the stat_com array
 *       and fragment buffers are not little-endian.
 */

/*
 *   V4L Buffer grabbing
 */
static int zoran_v4l_set_format(struct zoran *zr, int width, int height,
				const struct zoran_format *format)
{
	int bpp;

	/* Check size and format of the grab wanted */

	if (height < BUZ_MIN_HEIGHT || width < BUZ_MIN_WIDTH ||
	    height > BUZ_MAX_HEIGHT || width > BUZ_MAX_WIDTH) {
		pr_err("%s: %s - wrong frame size (%dx%d)\n", ZR_DEVNAME(zr), __func__, width, height);
		return -EINVAL;
	}

	bpp = (format->depth + 7) / 8;
#ifndef ZORAN_OLD
	/* TODO poison the last x bytes to verify overwrite*/
	zr->buffer_size = height * width * bpp + 4;
#endif
	pci_info(zr->pci_dev, "%s set %dx%d bpp=%d BUZ_MAX_HEIGHT=%d bufsize=%d\n", __func__,
		width, height, bpp, BUZ_MAX_HEIGHT, zr->buffer_size);

	/* Check against available buffer size */
	if (height * width * bpp > zr->buffer_size) {
		pr_err("%s: %s - video buffer size (%d kB) is too small\n",
			ZR_DEVNAME(zr), __func__, zr->buffer_size >> 10);
		return -EINVAL;
	}

	/* The video front end needs 4-byte alinged line sizes */

	if ((bpp == 2 && (width & 1)) || (bpp == 3 && (width & 3))) {
		pr_err("%s: %s - wrong frame alignment\n",
			ZR_DEVNAME(zr), __func__);
		return -EINVAL;
	}

	zr->v4l_settings.width = width;
	zr->v4l_settings.height = height;
	zr->v4l_settings.format = format;
	zr->v4l_settings.bytesperline = bpp * zr->v4l_settings.width;

	return 0;
}

static int setup_fbuffer(struct zoran *zr, void *base, const struct zoran_format *fmt,
			 int width, int height, int bytesperline)
{
	/* (Ronald) v4l/v4l2 guidelines */
	if (!capable(CAP_SYS_ADMIN) && !capable(CAP_SYS_RAWIO))
		return -EPERM;

	/* Don't allow frame buffer overlay if PCI or AGP is buggy, or on
	   ALi Magik (that needs very low latency while the card needs a
	   higher value always) */

	if (pci_pci_problems & (PCIPCI_FAIL | PCIAGP_FAIL | PCIPCI_ALIMAGIK))
		return -ENXIO;

	/* we need a bytesperline value, even if not given */
	if (!bytesperline)
		bytesperline = width * ((fmt->depth + 7) & ~7) / 8;

	if (height <= 0 || width <= 0 || bytesperline <= 0) {
		pr_err("%s: %s - invalid height/width/bpl value (%d|%d|%d)\n",
			ZR_DEVNAME(zr), __func__, width, height, bytesperline);
		return -EINVAL;
	}
	if (bytesperline & 3) {
		pr_err("%s: %s - bytesperline (%d) must be 4-byte aligned\n",
			ZR_DEVNAME(zr), __func__, bytesperline);
		return -EINVAL;
	}
	zr->vbuf_base = (void *)((unsigned long)base & ~3);
	zr->vbuf_height = height;
	zr->vbuf_width = width;
	zr->vbuf_depth = fmt->depth;
	zr->vbuf_bytesperline = bytesperline;

	return 0;
}

static int zoran_set_norm(struct zoran *zr, v4l2_std_id norm)
{

	if (!(norm & zr->card.norms)) {
		pr_err("%s: %s - unsupported norm %llx\n", ZR_DEVNAME(zr), __func__, norm);
		return -EINVAL;
	}

	if (norm & V4L2_STD_SECAM) {
		pci_info(zr->pci_dev, "Switching to SECAM\n");
		zr->timing = zr->card.tvn[ZR_NORM_SECAM];
	} else if (norm & V4L2_STD_NTSC) {
		pci_info(zr->pci_dev, "Switching to NTSC\n");
		zr->timing = zr->card.tvn[ZR_NORM_NTSC];
	} else {
		pci_info(zr->pci_dev, "Switching to PAL\n");
		zr->timing = zr->card.tvn[ZR_NORM_PAL];
	}

	pr_info("%s decoder_call s_std norm=%lld\n", __func__, norm);
	decoder_call(zr, video, s_std, norm);
	pr_info("%s decoder_call s_std_output norm=%lld\n", __func__, norm);
	encoder_call(zr, video, s_std_output, norm);

	/* Make sure the changes come into effect */
	zr->norm = norm;

	return 0;
}

static int zoran_set_input(struct zoran *zr, int input)
{
	pci_info(zr->pci_dev, "%s input=%d\n", __func__, input);
	if (input == zr->input)
		return 0;

	if (input < 0 || input >= zr->card.inputs) {
		pr_err("%s: %s - unsupported input %d\n", ZR_DEVNAME(zr), __func__, input);
		return -EINVAL;
	}

	zr->input = input;

	pr_info("%s decoder_call s_routing muxsel=%d 0 0\n", __func__, zr->card.input[input].muxsel);
	decoder_call(zr, video, s_routing, zr->card.input[input].muxsel, 0, 0);

	return 0;
}

/*
 *   ioctl routine
 */
static int zoran_querycap(struct file *file, void *__fh, struct v4l2_capability *cap)
{
	struct zoran *zr = video_drvdata(file);

	strscpy(cap->card, ZR_DEVNAME(zr), sizeof(cap->card));
	strscpy(cap->driver, "zoran", sizeof(cap->driver));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s", pci_name(zr->pci_dev));
	cap->device_caps = zr->video_dev->device_caps;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	pr_info("%s %x %x\n", __func__, cap->device_caps, cap->capabilities);
	return 0;
}

static int zoran_enum_fmt(struct zoran *zr, struct v4l2_fmtdesc *fmt, int flag)
{
	unsigned int num, i;

	if (fmt->index >= ARRAY_SIZE(zoran_formats))
		return -EINVAL;
	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	pci_info(zr->pci_dev, "%s: flag=%d type=%d idx=%d\n", __func__, flag, fmt->type, fmt->index);

	for (num = i = 0; i < NUM_FORMATS; i++) {
		if (zoran_formats[i].flags & flag && num++ == fmt->index) {
			pci_info(zr->pci_dev, "%s: Format %d %s\n", __func__, fmt->index, zoran_formats[i].name);
			strscpy(fmt->description, zoran_formats[i].name,
				sizeof(fmt->description));
			/* fmt struct pre-zeroed, so adding '\0' not needed */
			fmt->pixelformat = zoran_formats[i].fourcc;
			if (zoran_formats[i].flags & ZORAN_FORMAT_COMPRESSED)
				fmt->flags |= V4L2_FMT_FLAG_COMPRESSED;
			return 0;
		}
	}
	return -EINVAL;
}

static int zoran_enum_fmt_vid_cap(struct file *file, void *__fh,
				  struct v4l2_fmtdesc *f)
{
	struct zoran *zr = video_drvdata(file);

	return zoran_enum_fmt(zr, f, ZORAN_FORMAT_CAPTURE);
}

static int zoran_enum_fmt_vid_out(struct file *file, void *__fh,
				  struct v4l2_fmtdesc *f)
{
	struct zoran *zr = video_drvdata(file);

	return zoran_enum_fmt(zr, f, ZORAN_FORMAT_PLAYBACK);
}

static int zoran_g_fmt_vid_out(struct file *file, void *__fh,
			       struct v4l2_format *fmt)
{
	struct zoran *zr = video_drvdata(file);

	fmt->fmt.pix.width = zr->jpg_settings.img_width / zr->jpg_settings.HorDcm;
	fmt->fmt.pix.height = zr->jpg_settings.img_height * 2 /
		(zr->jpg_settings.VerDcm * zr->jpg_settings.TmpDcm);
	fmt->fmt.pix.sizeimage = zoran_v4l2_calc_bufsize(&zr->jpg_settings);
	fmt->fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	if (zr->jpg_settings.TmpDcm == 1)
		fmt->fmt.pix.field = (zr->jpg_settings.odd_even ?
				V4L2_FIELD_SEQ_TB : V4L2_FIELD_SEQ_BT);
	else
		fmt->fmt.pix.field = (zr->jpg_settings.odd_even ?
				V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM);
	fmt->fmt.pix.bytesperline = 0;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;

	pci_info(zr->pci_dev, "%s %dx%d size=%d fmt=%x Dcm=%d field=%x\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		fmt->fmt.pix.sizeimage, fmt->fmt.pix.pixelformat,
		zr->jpg_settings.TmpDcm,
		fmt->fmt.pix.field);
	return 0;
}

static int zoran_g_fmt_vid_cap(struct file *file, void *__fh,
			       struct v4l2_format *fmt)
{
	struct zoran *zr = video_drvdata(file);
	struct zoran_fh *fh = __fh;

	pci_info(zr->pci_dev, "%s\n", __func__);
	if (zr->map_mode != ZORAN_MAP_MODE_RAW)
		return zoran_g_fmt_vid_out(file, fh, fmt);
	fmt->fmt.pix.width = zr->v4l_settings.width;
	fmt->fmt.pix.height = zr->v4l_settings.height;
	fmt->fmt.pix.sizeimage = zr->v4l_settings.bytesperline * zr->v4l_settings.height;
	fmt->fmt.pix.pixelformat = zr->v4l_settings.format->fourcc;
	fmt->fmt.pix.colorspace = zr->v4l_settings.format->colorspace;
	fmt->fmt.pix.bytesperline = zr->v4l_settings.bytesperline;
	if (BUZ_MAX_HEIGHT < (zr->v4l_settings.height * 2))
		fmt->fmt.pix.field = V4L2_FIELD_INTERLACED;
	else
		fmt->fmt.pix.field = V4L2_FIELD_TOP;
	pci_info(zr->pci_dev, "%s using %dx%d sizeimage=%d pixelformat=%d colorspace=%d bytesperline=%d pixfiled=%d\n", __func__,
		zr->v4l_settings.width, zr->v4l_settings.height, fmt->fmt.pix.sizeimage, fmt->fmt.pix.pixelformat, fmt->fmt.pix.colorspace, fmt->fmt.pix.bytesperline, fmt->fmt.pix.field);
	return 0;
}

static int zoran_try_fmt_vid_out(struct file *file, void *__fh,
				 struct v4l2_format *fmt)
{
	struct zoran *zr = video_drvdata(file);
	struct zoran_jpg_settings settings;
	int res = 0;

	pci_info(zr->pci_dev, "%s\n", __func__);
	if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG)
		return -EINVAL;

	settings = zr->jpg_settings;

	/* we actually need to set 'real' parameters now */
	if ((fmt->fmt.pix.height * 2) > BUZ_MAX_HEIGHT)
		settings.TmpDcm = 1;
	else
		settings.TmpDcm = 2;
	settings.decimation = 0;
	if (fmt->fmt.pix.height <= zr->jpg_settings.img_height / 2)
		settings.VerDcm = 2;
	else
		settings.VerDcm = 1;
	if (fmt->fmt.pix.width <= zr->jpg_settings.img_width / 4)
		settings.HorDcm = 4;
	else if (fmt->fmt.pix.width <= zr->jpg_settings.img_width / 2)
		settings.HorDcm = 2;
	else
		settings.HorDcm = 1;
	if (settings.TmpDcm == 1)
		settings.field_per_buff = 2;
	else
		settings.field_per_buff = 1;

	if (settings.HorDcm > 1) {
		settings.img_x = (BUZ_MAX_WIDTH == 720) ? 8 : 0;
		settings.img_width = (BUZ_MAX_WIDTH == 720) ? 704 : BUZ_MAX_WIDTH;
	} else {
		settings.img_x = 0;
		settings.img_width = BUZ_MAX_WIDTH;
	}

	/* check */
	res = zoran_check_jpg_settings(zr, &settings, 1);
	if (res)
		return res;

	/* tell the user what we actually did */
	fmt->fmt.pix.width = settings.img_width / settings.HorDcm;
	fmt->fmt.pix.height = settings.img_height * 2 /
		(settings.TmpDcm * settings.VerDcm);
	if (settings.TmpDcm == 1)
		fmt->fmt.pix.field = (zr->jpg_settings.odd_even ?
				V4L2_FIELD_SEQ_TB : V4L2_FIELD_SEQ_BT);
	else
		fmt->fmt.pix.field = (zr->jpg_settings.odd_even ?
				V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM);

	fmt->fmt.pix.sizeimage = zoran_v4l2_calc_bufsize(&settings);
	fmt->fmt.pix.bytesperline = 0;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	return res;
}

static int zoran_try_fmt_vid_cap(struct file *file, void *__fh,
				 struct v4l2_format *fmt)
{
	struct zoran *zr = video_drvdata(file);
	int bpp;
	int i;

	pr_info("%s pixfmt=%x\n", __func__, fmt->fmt.pix.pixelformat);
	if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
		return zoran_try_fmt_vid_out(file, __fh, fmt);

	for (i = 0; i < NUM_FORMATS; i++)
		if (zoran_formats[i].fourcc == fmt->fmt.pix.pixelformat)
			break;

	if (i == NUM_FORMATS)
		return -EINVAL;

	fmt->fmt.pix.colorspace = zoran_formats[i].colorspace;
	if (BUZ_MAX_HEIGHT < (fmt->fmt.pix.height * 2))
		fmt->fmt.pix.field = V4L2_FIELD_INTERLACED;
	else
		fmt->fmt.pix.field = V4L2_FIELD_TOP;
	bpp = DIV_ROUND_UP(zoran_formats[i].depth, 8);
	v4l_bound_align_image(&fmt->fmt.pix.width, BUZ_MIN_WIDTH, BUZ_MAX_WIDTH, bpp == 2 ? 1 : 2,
		&fmt->fmt.pix.height, BUZ_MIN_HEIGHT, BUZ_MAX_HEIGHT, 0, 0);
	return 0;
}

static int zoran_s_fmt_vid_out(struct file *file, void *__fh,
			       struct v4l2_format *fmt)
{
	struct zoran *zr = video_drvdata(file);
	__le32 printformat = __cpu_to_le32(fmt->fmt.pix.pixelformat);
	struct zoran_jpg_settings settings;
	int res = 0;

	pci_info(zr->pci_dev, "%s: size=%dx%d, fmt=0x%x (%4.4s)\n",
		__func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
			fmt->fmt.pix.pixelformat,
			(char *)&printformat);
	if (fmt->fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
		pci_err(zr->pci_dev, "%s output support only V4L2_PIX_FMT_MJPEG %d\n", __func__, V4L2_PIX_FMT_MJPEG);
		return -EINVAL;
	}

	if (!fmt->fmt.pix.height || fmt->fmt.pix.width)
		return -EINVAL;

	settings = zr->jpg_settings;

	/* we actually need to set 'real' parameters now */
	if (fmt->fmt.pix.height * 2 > BUZ_MAX_HEIGHT)
		settings.TmpDcm = 1;
	else
		settings.TmpDcm = 2;
	settings.decimation = 0;
	if (fmt->fmt.pix.height <= zr->jpg_settings.img_height / 2)
		settings.VerDcm = 2;
	else
		settings.VerDcm = 1;
	if (fmt->fmt.pix.width <= zr->jpg_settings.img_width / 4)
		settings.HorDcm = 4;
	else if (fmt->fmt.pix.width <= zr->jpg_settings.img_width / 2)
		settings.HorDcm = 2;
	else
		settings.HorDcm = 1;
	if (settings.TmpDcm == 1)
		settings.field_per_buff = 2;
	else
		settings.field_per_buff = 1;

	if (settings.HorDcm > 1) {
		settings.img_x = (BUZ_MAX_WIDTH == 720) ? 8 : 0;
		settings.img_width = (BUZ_MAX_WIDTH == 720) ? 704 : BUZ_MAX_WIDTH;
	} else {
		settings.img_x = 0;
		settings.img_width = BUZ_MAX_WIDTH;
	}

	/* check */
	res = zoran_check_jpg_settings(zr, &settings, 0);
	if (res)
		return res;

	/* it's ok, so set them */
	zr->jpg_settings = settings;

	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		zr->map_mode = ZORAN_MAP_MODE_JPG_REC;
	else
		zr->map_mode = ZORAN_MAP_MODE_JPG_PLAY;

	zr->buffer_size = zoran_v4l2_calc_bufsize(&zr->jpg_settings);
	pci_info(zr->pci_dev, "%s MAPMODE is %s size=%d\n", __func__, map_mode(zr->map_mode), zr->buffer_size);

	/* tell the user what we actually did */
	fmt->fmt.pix.width = settings.img_width / settings.HorDcm;
	fmt->fmt.pix.height = settings.img_height * 2 /
		(settings.TmpDcm * settings.VerDcm);
	if (settings.TmpDcm == 1)
		fmt->fmt.pix.field = (zr->jpg_settings.odd_even ?
				V4L2_FIELD_SEQ_TB : V4L2_FIELD_SEQ_BT);
	else
		fmt->fmt.pix.field = (zr->jpg_settings.odd_even ?
				V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM);
	fmt->fmt.pix.bytesperline = 0;
	fmt->fmt.pix.sizeimage = zr->buffer_size;
	fmt->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
	return res;
}

static int zoran_s_fmt_vid_cap(struct file *file, void *__fh,
			       struct v4l2_format *fmt)
{
	struct zoran *zr = video_drvdata(file);
	struct zoran_fh *fh = __fh;
	int i;
	int res = 0;

	/* HACK */
	zr->buffer_size = 8192;

	if (fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG)
		return zoran_s_fmt_vid_out(file, fh, fmt);

	for (i = 0; i < NUM_FORMATS; i++)
		if (fmt->fmt.pix.pixelformat == zoran_formats[i].fourcc)
			break;
	if (i == NUM_FORMATS) {
		pr_err("%s: VIDIOC_S_FMT - unknown/unsupported format 0x%x\n",
			ZR_DEVNAME(zr), fmt->fmt.pix.pixelformat);
		return -EINVAL;
	}

	if (fmt->fmt.pix.height > BUZ_MAX_HEIGHT) {
		pci_info(zr->pci_dev, "%s limiting height to %d\n", __func__, BUZ_MAX_HEIGHT);
		fmt->fmt.pix.height = BUZ_MAX_HEIGHT;
	}
	if (fmt->fmt.pix.width > BUZ_MAX_WIDTH) {
		pci_info(zr->pci_dev, "%s limiting width to %d\n", __func__, BUZ_MAX_WIDTH);
		fmt->fmt.pix.width = BUZ_MAX_WIDTH;
	}

	zr->map_mode = ZORAN_MAP_MODE_RAW;
	pci_info(zr->pci_dev, "MAPMODE is %s\n", map_mode(zr->map_mode));

	res = zoran_v4l_set_format(zr, fmt->fmt.pix.width, fmt->fmt.pix.height,
				   &zoran_formats[i]);
	if (res)
		return res;

	/* tell the user the results/missing stuff */
	fmt->fmt.pix.bytesperline = zr->v4l_settings.bytesperline;
	fmt->fmt.pix.sizeimage = zr->v4l_settings.height * zr->v4l_settings.bytesperline;
	fmt->fmt.pix.colorspace = zr->v4l_settings.format->colorspace;
	if (BUZ_MAX_HEIGHT < (zr->v4l_settings.height * 2))
		fmt->fmt.pix.field = V4L2_FIELD_INTERLACED;
	else
		fmt->fmt.pix.field = V4L2_FIELD_TOP;
	return res;
}

static int zoran_g_fbuf(struct file *file, void *__fh,
			struct v4l2_framebuffer *fb)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s: fbuf is %dx%d\n", __func__, zr->vbuf_width, zr->vbuf_height);

	memset(fb, 0, sizeof(*fb));
	fb->base = zr->vbuf_base;
	fb->fmt.width = zr->vbuf_width;
	fb->fmt.height = zr->vbuf_height;
	fb->fmt.bytesperline = zr->vbuf_bytesperline;
	fb->fmt.colorspace = V4L2_COLORSPACE_SRGB;
	fb->fmt.field = V4L2_FIELD_INTERLACED;
	fb->capability = V4L2_FBUF_CAP_LIST_CLIPPING;

	return 0;
}

static int zoran_s_fbuf(struct file *file, void *__fh,
			const struct v4l2_framebuffer *fb)
{
	struct zoran *zr = video_drvdata(file);
	int i, res = 0;

	__le32 printformat = __cpu_to_le32(fb->fmt.pixelformat);

	for (i = 0; i < NUM_FORMATS; i++)
		if (zoran_formats[i].fourcc == fb->fmt.pixelformat)
			break;
	if (i == NUM_FORMATS) {
		pr_err("%s: VIDIOC_S_FBUF - format=0x%x (%4.4s) not allowed\n",
			ZR_DEVNAME(zr), fb->fmt.pixelformat,
			(char *)&printformat);
		return -EINVAL;
	}

	pci_info(zr->pci_dev, "%s: %dx%d bytesperline=%d\n", __func__, fb->fmt.width, fb->fmt.height, fb->fmt.bytesperline);

	res = setup_fbuffer(zr, fb->base, &zoran_formats[i], fb->fmt.width,
			    fb->fmt.height, fb->fmt.bytesperline);
	return res;
}

static int zoran_g_std(struct file *file, void *__fh, v4l2_std_id *std)
{
	struct zoran *zr = video_drvdata(file);

	switch(zr->norm) {
	case V4L2_STD_PAL:
		pci_info(zr->pci_dev, "%s using PAL (%lld)\n", __func__, zr->norm);
		break;
	case V4L2_STD_NTSC:
		pci_info(zr->pci_dev, "%s using NTSC (%lld)\n", __func__, zr->norm);
		break;
	case V4L2_STD_SECAM:
		pci_info(zr->pci_dev, "%s using SECAM (%lld)\n", __func__, zr->norm);
		break;
	default:
		pci_info(zr->pci_dev, "%s using unknow (%lld)\n", __func__, zr->norm);
		break;
	}

	*std = zr->norm;
	return 0;
}

static int zoran_s_std(struct file *file, void *__fh, v4l2_std_id std)
{
	struct zoran *zr = video_drvdata(file);
	int res = 0;

	pci_info(zr->pci_dev, "%s\n", __func__);
	res = zoran_set_norm(zr, std);
	if (res)
		return res;

	res = wait_grab_pending(zr);
	return res;
}

static int zoran_enum_input(struct file *file, void *__fh,
			    struct v4l2_input *inp)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s index=%d/%d\n", __func__, inp->index, zr->card.inputs);
	if (inp->index >= zr->card.inputs) {
		pci_err(zr->pci_dev, "%s input out of range %d >= %d\n", __func__, inp->index, zr->card.inputs);
		return -EINVAL;
	}

	strscpy(inp->name, zr->card.input[inp->index].name, sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM;
	pci_info(zr->pci_dev, "%s use %s\n", __func__, inp->name);

	/* Get status of video decoder */
	decoder_call(zr, video, g_input_status, &inp->status);
	pr_info("%s decoder_call g_input_status %d\n", __func__, inp->status);
	return 0;
}

static int zoran_g_input(struct file *file, void *__fh, unsigned int *input)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s input is %d\n", __func__, zr->input);
	*input = zr->input;
	return 0;
}

static int zoran_s_input(struct file *file, void *__fh, unsigned int input)
{
	struct zoran *zr = video_drvdata(file);
	int res;

	pci_info(zr->pci_dev, "%s set input=%d\n", __func__, input);
	res = zoran_set_input(zr, input);
	if (res)
		return res;

	/* Make sure the changes come into effect */
	res = wait_grab_pending(zr);
	return res;
}

static int zoran_enum_output(struct file *file, void *__fh,
			     struct v4l2_output *outp)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s index=%d\n", __func__, outp->index);
	if (outp->index != 0) {
		pci_err(zr->pci_dev, "%s output out of range\n", __func__);
		return -EINVAL;
	}

	outp->index = 0;
	outp->type = V4L2_OUTPUT_TYPE_ANALOGVGAOVERLAY;
	outp->std = V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM;
	outp->capabilities = V4L2_OUT_CAP_STD;
	strscpy(outp->name, "Autodetect", sizeof(outp->name));

	return 0;
}
static int zoran_g_output(struct file *file, void *__fh, unsigned int *output)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s\n", __func__);
	*output = 0;

	return 0;
}

static int zoran_s_output(struct file *file, void *__fh, unsigned int output)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s\n", __func__);
	if (output != 0) {
		pci_err(zr->pci_dev, "%s\n", __func__);
		return -EINVAL;
	}

	return 0;
}
/* cropping (sub-frame capture) */
static int zoran_g_selection(struct file *file, void *__fh, struct v4l2_selection *sel)
{
	struct zoran *zr = video_drvdata(file);

	pci_info(zr->pci_dev, "%s\n", __func__);
	if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pci_err(zr->pci_dev, "%s invalid combinaison\n", __func__);
		return -EINVAL;
	}

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r.top = zr->jpg_settings.img_y;
		sel->r.left = zr->jpg_settings.img_x;
		sel->r.width = zr->jpg_settings.img_width;
		sel->r.height = zr->jpg_settings.img_height;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = sel->r.left = 0;
		sel->r.width = BUZ_MIN_WIDTH;
		sel->r.height = BUZ_MIN_HEIGHT;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = sel->r.left = 0;
		sel->r.width = BUZ_MAX_WIDTH;
		sel->r.height = BUZ_MAX_HEIGHT;
		break;
	default:
		pci_err(zr->pci_dev, "%s Invalid target\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int zoran_s_selection(struct file *file, void *__fh, struct v4l2_selection *sel)
{
	struct zoran *zr = video_drvdata(file);
	struct zoran_jpg_settings settings;
	int res;

	pci_info(zr->pci_dev, "%s\n", __func__);
	if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if (zr->map_mode == ZORAN_MAP_MODE_RAW) {
		pr_err("%s: VIDIOC_S_SELECTION - subcapture only supported for compressed capture\n", ZR_DEVNAME(zr));
		return -EINVAL;
	}

	if (!sel->r.width || !sel->r.height)
		return -EINVAL;

	settings = zr->jpg_settings;

	/* move into a form that we understand */
	settings.img_x = sel->r.left;
	settings.img_y = sel->r.top;
	settings.img_width = sel->r.width;
	settings.img_height = sel->r.height;

	/* check validity */
	res = zoran_check_jpg_settings(zr, &settings, 0);
	if (res)
		return res;

	/* accept */
	zr->jpg_settings = settings;
	return res;
}

static int zoran_g_jpegcomp(struct file *file, void *__fh,
			    struct v4l2_jpegcompression *params)
{
	struct zoran *zr = video_drvdata(file);

	memset(params, 0, sizeof(*params));

	params->quality = zr->jpg_settings.jpg_comp.quality;
	params->APPn = zr->jpg_settings.jpg_comp.APPn;
	memcpy(params->APP_data, zr->jpg_settings.jpg_comp.APP_data,
	       zr->jpg_settings.jpg_comp.APP_len);
	params->APP_len = zr->jpg_settings.jpg_comp.APP_len;
	memcpy(params->COM_data, zr->jpg_settings.jpg_comp.COM_data,
	       zr->jpg_settings.jpg_comp.COM_len);
	params->COM_len = zr->jpg_settings.jpg_comp.COM_len;
	params->jpeg_markers = zr->jpg_settings.jpg_comp.jpeg_markers;
	pci_info(zr->pci_dev, "%s qty=%d APPn=%d\n", __func__, zr->jpg_settings.jpg_comp.quality,
		zr->jpg_settings.jpg_comp.APPn);

	return 0;
}

static int zoran_s_jpegcomp(struct file *file, void *__fh,
			    const struct v4l2_jpegcompression *params)
{
	struct zoran *zr = video_drvdata(file);
	int res = 0;
	struct zoran_jpg_settings settings;

	pr_info("%s\n", __func__);

	settings = zr->jpg_settings;

	settings.jpg_comp = *params;

	res = zoran_check_jpg_settings(zr, &settings, 0);
	if (res)
		return res;
	zr->buffer_size = zoran_v4l2_calc_bufsize(&zr->jpg_settings);
	zr->jpg_settings.jpg_comp = settings.jpg_comp;
	return res;
}

static int zoran_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *parm)
{
	struct v4l2_captureparm *cp = &parm->parm.capture;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/*cp->capability = V4L2_CAP_TIMEPERFRAME;*/
/*	cp->readbuffers = 2;*/

	return 0;
}

static const struct v4l2_ioctl_ops zoran_ioctl_ops = {
	.vidioc_querycap		    = zoran_querycap,
	.vidioc_s_selection		    = zoran_s_selection,
	.vidioc_g_selection		    = zoran_g_selection,
	.vidioc_enum_input		    = zoran_enum_input,
	.vidioc_g_input			    = zoran_g_input,
	.vidioc_s_input			    = zoran_s_input,
#ifndef COMPLIANCE
	.vidioc_enum_output		    = zoran_enum_output,
	.vidioc_g_output		    = zoran_g_output,
	.vidioc_s_output		    = zoran_s_output,
#endif
/*
	.vidioc_g_fbuf			    = zoran_g_fbuf,
	.vidioc_s_fbuf			    = zoran_s_fbuf,*/
	.vidioc_g_std			    = zoran_g_std,
	.vidioc_s_std			    = zoran_s_std,
	.vidioc_g_jpegcomp		    = zoran_g_jpegcomp,
	.vidioc_s_jpegcomp		    = zoran_s_jpegcomp,
	.vidioc_reqbufs			    = vb2_ioctl_reqbufs,
	.vidioc_querybuf		    = vb2_ioctl_querybuf,
	.vidioc_qbuf			    = vb2_ioctl_qbuf,
	.vidioc_dqbuf			    = vb2_ioctl_dqbuf,
	.vidioc_expbuf                      = vb2_ioctl_expbuf,
	.vidioc_streamon		    = vb2_ioctl_streamon,
	.vidioc_streamoff		    = vb2_ioctl_streamoff,
	.vidioc_g_parm			    = zoran_g_parm,
	.vidioc_enum_fmt_vid_cap            = zoran_enum_fmt_vid_cap,
/*	.vidioc_enum_fmt_vid_out            = zoran_enum_fmt_vid_out,*/
	.vidioc_g_fmt_vid_cap               = zoran_g_fmt_vid_cap,
/*	.vidioc_g_fmt_vid_out               = zoran_g_fmt_vid_out,*/
	.vidioc_s_fmt_vid_cap               = zoran_s_fmt_vid_cap,
/*	.vidioc_s_fmt_vid_out               = zoran_s_fmt_vid_out,*/
	.vidioc_try_fmt_vid_cap             = zoran_try_fmt_vid_cap,
/*	.vidioc_try_fmt_vid_out             = zoran_try_fmt_vid_out,*/
	.vidioc_subscribe_event             = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event           = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations zoran_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
       .open		= v4l2_fh_open,
       .release		= vb2_fop_release,
       .read		= vb2_fop_read,
       .write		= vb2_fop_write,
       .mmap		= vb2_fop_mmap,
       .poll		= vb2_fop_poll,
};

const struct video_device zoran_template = {
	.name = ZORAN_NAME,
	.fops = &zoran_fops,
	.ioctl_ops = &zoran_ioctl_ops,
	.release = &zoran_vdev_release,
	.tvnorms = V4L2_STD_NTSC | V4L2_STD_PAL | V4L2_STD_SECAM,
};

























static int zr_vout_vb2_queue_setup(struct vb2_queue *vq,
		unsigned int *nbuffers, unsigned int *nplanes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	struct zoran *zr = vb2_get_drv_priv(vq);
	unsigned int size = PAGE_ALIGN(zr->buffer_size);

	if (!size) {
		zr->buffer_size = 500000;
		size = PAGE_ALIGN(zr->buffer_size);
	}

	pr_info("%s nplanes=%d size=%u nbuffers=%d\n", __func__, *nplanes, size, *nbuffers);

	/*if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;*/
	if (*nplanes) {
		pr_info("%s size0=%d size=%d\n", __func__, sizes[0], size);
	}
	*nplanes = 1;
	sizes[0] = size;

	zr->buf_in_reserve = 0;

	return 0;
}

static unsigned int zr_jpgclean(struct vb2_buffer *vb)
{
	struct zoran *zr = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long i, t;
	u8 *j;
	int size;
	unsigned long payload;
	unsigned long last;

	payload = vb2_get_plane_payload(vb, 0);
	pr_info("Check buffer size=%lu\n", payload);
	payload = zr->buffer_size;

	j = vb2_plane_vaddr(vb, 0);
	if (j[0] != 0xff || j[1] != 0xd8)
		return 0;

	if (!zr->blobo.size) {
		memcpy(zr->frameo, j, zr->buffer_size);
		zr->blobo.size = zr->buffer_size;
		zr->blobo.data = zr->frameo;
	}

	t = 2;
	i = 2;
	while (i < payload) {
		if (j[i] == 0xff && (j[i + 1] == 0xe0 || j[i + 1] == 0xfe)) {
			size = j[i + 2] * 256 + j[i + 3] + 2;
			i += size;
/*			pr_info("SKIP jpeg at %d size=%d t=%d\n", i, size, t);*/
			continue;
		}
		if (j[i] == 0xff && j[i + 1] == 0xd9)
			last = t + 1;
		if (i != t)
			j[t] = j[i];
		i++;
		t++;
	}
	if (payload != t) {
		pr_info("%s clean jpeg from %lu to %lu %lu\n", __func__, payload, t, last);
		vb2_set_plane_payload(vb, 0, last);
	}
	if (!zr->blobn.size) {
		memcpy(zr->framen, j, zr->buffer_size);
		zr->blobn.size = zr->buffer_size;
		zr->blobn.data = zr->framen;
	}

	return t;
}
static void zr_vout_vb2_queue(struct vb2_buffer *vb)
{
	struct zoran *zr = vb2_get_drv_priv(vb->vb2_queue);
	struct zr_vout_buffer *voutbuf = vb2_to_zr_vout_buffer(vb);
	unsigned long flags;

	if (zr->map_mode == ZORAN_MAP_MODE_JPG_REC)
		zr_jpgclean(vb);

	pci_info(zr->pci_dev, "%s %d\n", __func__, zr->running);
	spin_lock_irqsave(&zr->queued_bufs_lock, flags);
	list_add_tail(&voutbuf->queue, &zr->queued_bufs);
	zr->buf_in_reserve++;
	spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
	if (zr->running == ZORAN_MAP_MODE_JPG_REC)
		zoran_feed_stat_com(zr);
	else
		pci_info(zr->pci_dev, "Not feeding\n");
	zr->queued++;
}

static int zr_vout_vb2_prepare(struct vb2_buffer *vb)
{
	struct zoran *zr = vb2_get_drv_priv(vb->vb2_queue);

	pci_info(zr->pci_dev, "%s\n", __func__);

	if (vb2_plane_size(vb, 0) < 10)
		pr_warn("BAD SIZE\n");

	/* poison */
	/*memset(vb2_plane_vaddr(vb, 0), 0x66, zr->buffer_size);*/
	zr->prepared++;
	return 0;
}

/* TODO seq */
static unsigned int seq;

static int zr_set_jpgbuf(struct zoran *zr)
{
	int i;
	pr_info("%s: queue_state=%ld/%ld/%ld/%ld\n",
		__func__, zr->jpg_que_tail, zr->jpg_dma_tail,
		zr->jpg_dma_head, zr->jpg_que_head);
	for (i = 0; i < BUZ_NUM_STAT_COM; i++)
		pr_info("%s: stat_com %d %ux\n", __func__, i, zr->stat_com[i]);
	return 0;
}

int zr_set_buf(struct zoran *zr)
{
	struct zr_vout_buffer *buf;
	struct vb2_v4l2_buffer *vbuf;
	dma_addr_t phys_addr;
	unsigned long flags;
	u32 reg;

	if (zr->running == ZORAN_MAP_MODE_NONE) {
		pr_info("%s not init\n", __func__);
		return 0;
	}

	if (zr->inuse[0]) {
		buf = zr->inuse[0];
		buf->vbuf.vb2_buf.timestamp = ktime_get_ns();
		buf->vbuf.sequence = seq++;
		vbuf = &buf->vbuf;

		/* TODO field*/
		buf->vbuf.field = V4L2_FIELD_INTERLACED;
		vb2_set_plane_payload(&buf->vbuf.vb2_buf, 0, zr->buffer_size);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_DONE);
		zr->inuse[0] = NULL;
	}

	spin_lock_irqsave(&zr->queued_bufs_lock, flags);
	if (list_empty(&zr->queued_bufs)) {
		pr_err("%s: not buf to use\n", __func__);
		btand(~ZR36057_ICR_IntPinEn, ZR36057_ICR);
		vb2_queue_error(zr->video_dev->queue);
		spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
		return -EINVAL;
	}
	buf = list_first_entry_or_null(&zr->queued_bufs, struct zr_vout_buffer, queue);
	if (!buf) {
		btand(~ZR36057_ICR_IntPinEn, ZR36057_ICR);
		vb2_queue_error(zr->video_dev->queue);
		spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
		return -EINVAL;
	}
	list_del(&buf->queue);
	spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);

	vbuf = &buf->vbuf;
	vbuf->vb2_buf.state = VB2_BUF_STATE_ACTIVE;
	/*memset(vb2_plane_vaddr(&vbuf->vb2_buf, 0), 0x66, zr->buffer_size);*/
	phys_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);

	if (!phys_addr) {
		pr_err("ERROR: no DMA addr\n");
		return -EINVAL;
	}

	zr->inuse[0] = buf;

	reg = phys_addr;
	btwrite(reg, ZR36057_VDTR);
	if (zr->v4l_settings.height > BUZ_MAX_HEIGHT / 2)
		reg += zr->v4l_settings.bytesperline;
	btwrite(reg, ZR36057_VDBR);
	pr_info("%s sizehack heigth=%d BUZ_MAX_HEIGHT=%d bytesperline=%d\n", __func__, zr->v4l_settings.height, BUZ_MAX_HEIGHT, zr->v4l_settings.bytesperline);

	reg = 0;
	if (zr->v4l_settings.height > BUZ_MAX_HEIGHT / 2)
		reg += zr->v4l_settings.bytesperline;
	reg = (reg << ZR36057_VSSFGR_DispStride);
	reg |= ZR36057_VSSFGR_VidOvf;
	reg |= ZR36057_VSSFGR_SnapShot;
	reg |= ZR36057_VSSFGR_FrameGrab;
	btwrite(reg, ZR36057_VSSFGR);

	btor(ZR36057_VDCR_VidEn, ZR36057_VDCR);
	return 0;
}

static int zr_vout_vb2_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct zoran *zr = vq->drv_priv;
	int j;

	pr_info("%s\n", __func__);
	for (j = 0; j < BUZ_NUM_STAT_COM; j++) {
		zr->stat_com[j] = cpu_to_le32(1);
		zr->inuse[j] = 0;
	}

	pci_info(zr->pci_dev, "%s MAPMODE is %s\n", __func__, map_mode(zr->map_mode));
	if (zr->map_mode != ZORAN_MAP_MODE_RAW) {
		pci_info(zr->pci_dev, "START JPG\n");
		zr36057_restart(zr);
		zoran_init_hardware(zr);
/*	zr36057_restart(zr);*/
		if (zr->map_mode == ZORAN_MAP_MODE_JPG_REC)
			zr36057_enable_jpg(zr, BUZ_MODE_MOTION_DECOMPRESS);
		else
			zr36057_enable_jpg(zr, BUZ_MODE_MOTION_COMPRESS);
		zoran_feed_stat_com(zr);
		jpeg_start(zr);
		btor(ZR36057_ICR_IntPinEn, ZR36057_ICR);
		zr->running = zr->map_mode;
		return 0;
	}
	pci_info(zr->pci_dev, "START RAW\n");
	zr36057_restart(zr);
	zoran_init_hardware(zr);

/*	zoran_set_norm(zr, zr->norm);*/

	zr36057_enable_jpg(zr, BUZ_MODE_IDLE);
	zr36057_set_memgrab(zr, 1);
	btor(ZR36057_ICR_IntPinEn, ZR36057_ICR);
	zr->running = zr->map_mode;
	zoran_status(zr);
	return 0;
}

static void zr_vout_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct zoran *zr = vq->drv_priv;
	struct zr_vout_buffer *buf;
	unsigned long flags;
	int j;

	zr->running = ZORAN_MAP_MODE_NONE;
	pci_info(zr->pci_dev, "%s\n", __func__);

	btand(~ZR36057_ICR_IntPinEn, ZR36057_ICR);
	if (zr->map_mode != ZORAN_MAP_MODE_RAW) {
		zr36057_enable_jpg(zr, BUZ_MODE_IDLE);
		zr36057_set_memgrab(zr, 0);
	} else {
		zr36057_set_memgrab(zr, 0);
	}

	zoran_set_pci_master(zr, 0);

	if (!pass_through) {	/* Switch to color bar */
		pr_info("%s decoder_call s_stream 0\n", __func__);
		decoder_call(zr, video, s_stream, 0);
		pr_info("%s encoder_call s_routing 2 0 0\n", __func__);
		encoder_call(zr, video, s_routing, 2, 0, 0);
	}


	for (j = 0; j < BUZ_NUM_STAT_COM; j++) {
		zr->stat_com[j] = cpu_to_le32(1);
		if (!zr->inuse[j])
			continue;
		buf = zr->inuse[j];
		pci_info(zr->pci_dev, "%s clean %d %px\n", __func__, j, buf);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
		zr->inuse[j] = NULL;
	}

	spin_lock_irqsave(&zr->queued_bufs_lock, flags);
	while (!list_empty(&zr->queued_bufs)) {
		buf = list_entry(zr->queued_bufs.next, struct zr_vout_buffer, queue);
		list_del(&buf->queue);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
		zr->buf_in_reserve--;
	}
	spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
	if (zr->buf_in_reserve)
		pci_err(zr->pci_dev, "Buffer remaining %d\n", zr->buf_in_reserve);
	zr->map_mode = ZORAN_MAP_MODE_RAW;
}

const struct vb2_ops zr_video_qops = {
	.queue_setup            = zr_vout_vb2_queue_setup,
	.buf_queue              = zr_vout_vb2_queue,
	.buf_prepare            = zr_vout_vb2_prepare,
	.start_streaming        = zr_vout_vb2_start_streaming,
	.stop_streaming         = zr_vout_vb2_stop_streaming,
	.wait_prepare           = vb2_ops_wait_prepare,
	.wait_finish            = vb2_ops_wait_finish,
};

int zoran_queue_init(struct zoran *zr, struct vb2_queue *vq)
{
	int err;

	pr_info("%s %px %px\n", __func__, zr, vq);
	vq->dev = &zr->pci_dev->dev;
/*	vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE | V4L2_BUF_TYPE_VIDEO_OUTPUT;*/
	vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vq->io_modes = VB2_USERPTR | VB2_DMABUF | VB2_MMAP | VB2_READ | VB2_WRITE;
	vq->drv_priv = zr;
	vq->buf_struct_size = sizeof(struct zr_vout_buffer);
	vq->ops = &zr_video_qops;
	vq->mem_ops = &vb2_dma_contig_memops;
	vq->gfp_flags = GFP_DMA32,
	vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vq->min_buffers_needed = 9;
	vq->lock = &zr->lock;
	err = vb2_queue_init(vq);
	if (err)
		return err;
	zr->video_dev->queue = vq;
	return 0;
}
