// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Zoran zr36057/zr36067 PCI controller driver, for the
 * Pinnacle/Miro DC10/DC10+/DC30/DC30+, Iomega Buz, Linux
 * Media Labs LML33/LML33R10.
 *
 * This part handles device access (PCI/I2C/codec/...)
 *
 * Copyright (C) 2000 Serguei Miridonov <mirsev@cicese.mx>
 *
 * Currently maintained by:
 *   Ronald Bultje    <rbultje@ronald.bitfreak.net>
 *   Laurent Pinchart <laurent.pinchart@skynet.be>
 *   Mailinglist      <mjpeg-users@lists.sf.net>
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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/ktime.h>
#include <linux/sched/signal.h>

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <linux/spinlock.h>
#include <linux/sem.h>

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>

#include <asm/byteorder.h>
#include <linux/io.h>

#include "videocodec.h"
#include "zoran.h"
#include "zoran_device.h"
#include "zoran_card.h"

#define IRQ_MASK (ZR36057_ISR_GIRQ0 | \
		  ZR36057_ISR_GIRQ1 | \
		  ZR36057_ISR_JPEGRepIRQ)

static bool lml33dpath;		/* default = 0
				 * 1 will use digital path in capture
				 * mode instead of analog. It can be
				 * used for picture adjustments using
				 * tool like xawtv while watching image
				 * on TV monitor connected to the output.
				 * However, due to absence of 75 Ohm
				 * load on Bt819 input, there will be
				 * some image imperfections
				 */

module_param(lml33dpath, bool, 0644);
MODULE_PARM_DESC(lml33dpath, "Use digital path capture mode (on LML33 cards)");

int zr_set_buf(struct zoran *zr);
int zoran_status(struct zoran *zr);

/*
 * initialize video front end
 */
static void zr36057_init_vfe(struct zoran *zr)
{
	u32 reg;

	reg = btread(ZR36057_VFESPFR);
	reg |= ZR36057_VFESPFR_LittleEndian;
	reg &= ~ZR36057_VFESPFR_VCLKPol;
	reg |= ZR36057_VFESPFR_ExtFl;
	reg |= ZR36057_VFESPFR_TopField;
	btwrite(reg, ZR36057_VFESPFR);

	reg = btread(ZR36057_VDCR);
	if (pci_pci_problems & PCIPCI_TRITON)
		// || zr->revision < 1) // Revision 1 has also Triton support
		reg &= ~ZR36057_VDCR_Triton;
	else
		reg |= ZR36057_VDCR_Triton;
	btwrite(reg, ZR36057_VDCR);
}

/*
 * General Purpose I/O and Guest bus access
 */

/*
 * This is a bit tricky. When a board lacks a GPIO function, the corresponding
 * GPIO bit number in the card_info structure is set to 0.
 */

void GPIO(struct zoran *zr, int bit, unsigned int value)
{
	u32 reg;
	u32 mask;

	pr_info("%s %x %x\n", __func__, bit, value);
	/* Make sure the bit number is legal
	 * A bit number of -1 (lacking) gives a mask of 0,
	 * making it harmless
	 */
	mask = (1 << (24 + bit)) & 0xff000000;
	reg = btread(ZR36057_GPPGCR1) & ~mask;
	if (value)
		reg |= mask;

	btwrite(reg, ZR36057_GPPGCR1);
	udelay(1);
}

/*
 * Wait til post office is no longer busy
 */

int post_office_wait(struct zoran *zr)
{
	u32 por;

//      while (((por = btread(ZR36057_POR)) & (ZR36057_POR_POPen | ZR36057_POR_POTime)) == ZR36057_POR_POPen) {
	while ((por = btread(ZR36057_POR)) & ZR36057_POR_POPen) {
		/* wait for something to happen */
		/* TODO add timeout */
	}
	if ((por & ZR36057_POR_POTime) && !zr->card.gws_not_connected) {
		/* In LML33/BUZ \GWS line is not connected, so it has always timeout set */
		dprintk(1, KERN_INFO "%s: pop timeout %08x\n", ZR_DEVNAME(zr),
			por);
		return -1;
	}

	return 0;
}

int post_office_write(struct zoran *zr, unsigned int guest,
		      unsigned int reg, unsigned int value)
{
	u32 por;

	por =
	    ZR36057_POR_PODir | ZR36057_POR_POTime | ((guest & 7) << 20) |
	    ((reg & 7) << 16) | (value & 0xFF);
	btwrite(por, ZR36057_POR);

	return post_office_wait(zr);
}

int post_office_read(struct zoran *zr, unsigned int guest, unsigned int reg)
{
	u32 por;

	por = ZR36057_POR_POTime | ((guest & 7) << 20) | ((reg & 7) << 16);
	btwrite(por, ZR36057_POR);
	if (post_office_wait(zr) < 0)
		return -1;

	return btread(ZR36057_POR) & 0xFF;
}

/*
 * detect guests
 */

static void dump_guests(struct zoran *zr)
{
	if (zr36067_debug > 2) {
		int i, guest[8];

		/* do not print random data */
		guest[0] = 0;

		for (i = 1; i < 8; i++) /* Don't read jpeg codec here */
			guest[i] = post_office_read(zr, i, 0);

		pr_info("%s: Guests: %*ph\n", ZR_DEVNAME(zr), 8, guest);
	}
}

void detect_guest_activity(struct zoran *zr)
{
	int timeout, i, j, res, guest[8], guest0[8], change[8][3];
	ktime_t t0, t1;
	
	/* do not print random data */
	guest[0] = 0;
	guest0[0] = 0;

	dump_guests(zr);
	pr_info("%s: Detecting guests activity, please wait...\n", ZR_DEVNAME(zr));
	for (i = 1; i < 8; i++) /* Don't read jpeg codec here */
		guest0[i] = guest[i] = post_office_read(zr, i, 0);

	timeout = 0;
	j = 0;
	t0 = ktime_get();
	while (timeout < 10000) {
		udelay(10);
		timeout++;
		for (i = 1; (i < 8) && (j < 8); i++) {
			res = post_office_read(zr, i, 0);
			if (res != guest[i]) {
				t1 = ktime_get();
				change[j][0] = ktime_to_us(ktime_sub(t1, t0));
				t0 = t1;
				change[j][1] = i;
				change[j][2] = res;
				j++;
				guest[i] = res;
			}
		}
		if (j >= 8)
			break;
	}

	pr_info("%s: Guests: %*ph\n", ZR_DEVNAME(zr), 8, guest0);

	if (j == 0) {
		pr_info("%s: No activity detected.\n", ZR_DEVNAME(zr));
		return;
	}
	for (i = 0; i < j; i++)
		pr_info("%s: %6d: %d => 0x%02x\n", ZR_DEVNAME(zr),
			change[i][0], change[i][1], change[i][2]);
}

/*
 * JPEG Codec access
 */

void jpeg_codec_sleep(struct zoran *zr, int sleep)
{
	GPIO(zr, zr->card.gpio[ZR_GPIO_JPEG_SLEEP], !sleep);
	if (!sleep) {
		dprintk(3,
			KERN_INFO
			"%s: %s() - wake GPIO=0x%08x\n",
			ZR_DEVNAME(zr), __func__, btread(ZR36057_GPPGCR1));
		udelay(500);
	} else {
		dprintk(3,
			KERN_INFO
			"%s: %s() - sleep GPIO=0x%08x\n",
			ZR_DEVNAME(zr), __func__, btread(ZR36057_GPPGCR1));
		udelay(2);
	}
}

int jpeg_codec_reset(struct zoran *zr)
{
	/* Take the codec out of sleep */
	jpeg_codec_sleep(zr, 0);

	if (zr->card.gpcs[GPCS_JPEG_RESET] != 0xff) {
		post_office_write(zr, zr->card.gpcs[GPCS_JPEG_RESET], 0,
				  0);
		udelay(2);
	} else {
		GPIO(zr, zr->card.gpio[ZR_GPIO_JPEG_RESET], 0);
		udelay(2);
		GPIO(zr, zr->card.gpio[ZR_GPIO_JPEG_RESET], 1);
		udelay(2);
	}

	return 0;
}

/*
 *   Set the registers for the size we have specified. Don't bother
 *   trying to understand this without the ZR36057 manual in front of
 *   you [AC].
 */

static void zr36057_adjust_vfe(struct zoran *zr, enum zoran_codec_mode  mode)
{
	u32 reg;

	switch (mode) {
	case BUZ_MODE_MOTION_DECOMPRESS:
		btand(~ZR36057_VFESPFR_ExtFl, ZR36057_VFESPFR);
		reg = btread(ZR36057_VFEHCR);
		if ((reg & (1 << 10)) && zr->card.type != LML33R10)
			reg += ((1 << 10) | 1);

		btwrite(reg, ZR36057_VFEHCR);
		pr_info("%s mode=%d ZR36057_VFEHCR %x\n", __func__, mode, reg);
		break;
	case BUZ_MODE_MOTION_COMPRESS:
	case BUZ_MODE_IDLE:
	default:
		if ((zr->norm & V4L2_STD_NTSC) ||
		    (zr->card.type == LML33R10 &&
		     (zr->norm & V4L2_STD_PAL))) {
			btand(~ZR36057_VFESPFR_ExtFl, ZR36057_VFESPFR);
		} else {
			btor(ZR36057_VFESPFR_ExtFl, ZR36057_VFESPFR); }
		reg = btread(ZR36057_VFEHCR);
		if (!(reg & (1 << 10)) && zr->card.type != LML33R10)
			reg -= ((1 << 10) | 1);

		btwrite(reg, ZR36057_VFEHCR);
		pr_info("%s mode=%d default ZR36057_VFEHCR %x\n", __func__, mode, reg);
		break;
	}
}

/*
 * set geometry
 */

static void zr36057_set_vfe(struct zoran *zr, int video_width, int video_height,
			    const struct zoran_format *format)
{
	const struct tvnorm *tvn;
	unsigned int HStart, HEnd, VStart, VEnd;
	unsigned int DispMode;
	unsigned int VidWinWid, VidWinHt;
	unsigned int hcrop1, hcrop2, vcrop1, vcrop2;
	unsigned int Wa, We, Ha, He;
	unsigned int X, Y, HorDcm, VerDcm;
	u32 reg;

	pr_info("%s %dx%d\n", __func__, video_width, video_height);
	tvn = zr->timing;

	Wa = tvn->Wa;
	Ha = tvn->Ha;

	dprintk(2, KERN_INFO "%s: set_vfe() - width = %d, height = %d\n",
		ZR_DEVNAME(zr), video_width, video_height);

	if (video_width < BUZ_MIN_WIDTH ||
	    video_height < BUZ_MIN_HEIGHT ||
	    video_width > Wa || video_height > Ha) {
		pr_err("%s: set_vfe: w=%d h=%d not valid\n",
			ZR_DEVNAME(zr), video_width, video_height);
		return;
	}

	/**** zr36057 ****/

	/* horizontal */
	VidWinWid = video_width;
	X = DIV_ROUND_UP(VidWinWid * 64, tvn->Wa);
	We = (VidWinWid * 64) / X;
	HorDcm = 64 - X;
	hcrop1 = 2 * ((tvn->Wa - We) / 4);
	hcrop2 = tvn->Wa - We - hcrop1;
	HStart = tvn->HStart ? tvn->HStart : 1;
	/* (Ronald) Original comment:
	 * "| 1 Doesn't have any effect, tested on both a DC10 and a DC10+"
	 * this is false. It inverses chroma values on the LML33R10 (so Cr
	 * suddenly is shown as Cb and reverse, really cool effect if you
	 * want to see blue faces, not useful otherwise). So don't use |1.
	 * However, the DC10 has '0' as HStart, but does need |1, so we
	 * use a dirty check...
	 */
	HEnd = HStart + tvn->Wa - 1;
	HStart += hcrop1;
	HEnd -= hcrop2;
	reg = ((HStart & ZR36057_VFEHCR_Hmask) << ZR36057_VFEHCR_HStart)
	    | ((HEnd & ZR36057_VFEHCR_Hmask) << ZR36057_VFEHCR_HEnd);
	if (zr->card.vfe_pol.hsync_pol)
		reg |= ZR36057_VFEHCR_HSPol;
	btwrite(reg, ZR36057_VFEHCR);

	/* Vertical */
	DispMode = !(video_height > BUZ_MAX_HEIGHT / 2);
	VidWinHt = DispMode ? video_height : video_height / 2;
	Y = DIV_ROUND_UP(VidWinHt * 64 * 2, tvn->Ha);
	He = (VidWinHt * 64) / Y;
	VerDcm = 64 - Y;
	vcrop1 = (tvn->Ha / 2 - He) / 2;
	vcrop2 = tvn->Ha / 2 - He - vcrop1;
	VStart = tvn->VStart;
	VEnd = VStart + tvn->Ha / 2;	// - 1; FIXME SnapShot times out with -1 in 768*576 on the DC10 - LP
	VStart += vcrop1;
	VEnd -= vcrop2;
	reg = ((VStart & ZR36057_VFEVCR_Vmask) << ZR36057_VFEVCR_VStart)
	    | ((VEnd & ZR36057_VFEVCR_Vmask) << ZR36057_VFEVCR_VEnd);
	if (zr->card.vfe_pol.vsync_pol)
		reg |= ZR36057_VFEVCR_VSPol;
	btwrite(reg, ZR36057_VFEVCR);

	/* scaler and pixel format */
	reg = 0;
	reg |= (HorDcm << ZR36057_VFESPFR_HorDcm);
	reg |= (VerDcm << ZR36057_VFESPFR_VerDcm);
	reg |= (DispMode << ZR36057_VFESPFR_DispMode);
	/* RJ: I don't know, why the following has to be the opposite
	 * of the corresponding ZR36060 setting, but only this way
	 * we get the correct colors when uncompressing to the screen  */
	//reg |= ZR36057_VFESPFR_VCLKPol; /**/
	/* RJ: Don't know if that is needed for NTSC also */
	if (!(zr->norm & V4L2_STD_NTSC))
		reg |= ZR36057_VFESPFR_ExtFl;	// NEEDED!!!!!!! Wolfgang
	reg |= ZR36057_VFESPFR_TopField;
	if (HorDcm >= 48)
		reg |= 3 << ZR36057_VFESPFR_HFilter;	/* 5 tap filter */
	else if (HorDcm >= 32)
		reg |= 2 << ZR36057_VFESPFR_HFilter;	/* 4 tap filter */
	else if (HorDcm >= 16)
		reg |= 1 << ZR36057_VFESPFR_HFilter;	/* 3 tap filter */

	reg |= format->vfespfr;
	btwrite(reg, ZR36057_VFESPFR);

	/* display configuration */
	reg = (16 << ZR36057_VDCR_MinPix)
	    | (VidWinHt << ZR36057_VDCR_VidWinHt)
	    | (VidWinWid << ZR36057_VDCR_VidWinWid);
	if (pci_pci_problems & PCIPCI_TRITON)
		// || zr->revision < 1) // Revision 1 has also Triton support
		reg &= ~ZR36057_VDCR_Triton;
	else
		reg |= ZR36057_VDCR_Triton;
	btwrite(reg, ZR36057_VDCR);

	zr36057_adjust_vfe(zr, zr->codec_mode);
}

/* Enable/Disable uncompressed memory grabbing of the 36057 */
void zr36057_set_memgrab(struct zoran *zr, int mode)
{
	pci_info(zr->pci_dev, "%s: mode=%d\n", __func__, mode);
	if (mode) {
		/* We only check SnapShot and not FrameGrab here.  SnapShot==1
		 * means a capture is already in progress, but FrameGrab==1
		 * doesn't necessary mean that.  It's more correct to say a 1
		 * to 0 transition indicates a capture completed.  If a
		 * capture is pending when capturing is tuned off, FrameGrab
		 * will be stuck at 1 until capturing is turned back on.
		 */
		if (btread(ZR36057_VSSFGR) & ZR36057_VSSFGR_SnapShot)
			pr_warn("%s: zr36057_set_memgrab(1) with SnapShot on!?\n",
				ZR_DEVNAME(zr));

		/* switch on VSync interrupts */
		btwrite(IRQ_MASK, ZR36057_ISR);
		btor(ZR36057_ICR_IntPinEn, ZR36057_ICR);
		btor(zr->card.vsync_int, ZR36057_ICR);	// SW
		/* enable SnapShot */
		btor(ZR36057_VSSFGR_SnapShot, ZR36057_VSSFGR);

		/* Set zr36057 video front end  and enable video */
		zr36057_set_vfe(zr, zr->v4l_settings.width,
				zr->v4l_settings.height,
				zr->v4l_settings.format);

		zr->v4l_memgrab_active = 1;
	} else {
		/* switch off VSync interrupts */
		btand(~zr->card.vsync_int, ZR36057_ICR);	// SW

		zr->v4l_memgrab_active = 0;
		zr->v4l_grab_frame = NO_GRAB_ACTIVE;

		btand(~ZR36057_VDCR_VidEn, ZR36057_VDCR);
		btand(~ZR36057_VSSFGR_SnapShot, ZR36057_VSSFGR);
	}
}

int wait_grab_pending(struct zoran *zr)
{

	pr_info("%s\n", __func__);
	/* wait until all pending grabs are finished */
	return 0;
}

/*****************************************************************************
 *                                                                           *
 *  Set up the Buz-specific MJPEG part                                       *
 *                                                                           *
 *****************************************************************************/

static inline void set_frame(struct zoran *zr, int val)
{
	GPIO(zr, zr->card.gpio[ZR_GPIO_JPEG_FRAME], val);
}

static void set_videobus_dir(struct zoran *zr, int val)
{
	pr_info("%s val=%d\n", __func__, val);
	switch (zr->card.type) {
	case LML33:
	case LML33R10:
		if (!lml33dpath)
			GPIO(zr, 5, val);
		else
			GPIO(zr, 5, 1);
		break;
	default:
		GPIO(zr, zr->card.gpio[ZR_GPIO_VID_DIR],
		     zr->card.gpio_pol[ZR_GPIO_VID_DIR] ? !val : val);
		break;
	}
}

static void init_jpeg_queue(struct zoran *zr)
{
	int i;

	pr_info("%s\n", __func__);
/*	pci_info(zr->pci_dev, "%s num=%d\n", __func__, zr->jpg_buffers.num_buffers);*/
	/* re-initialize DMA ring stuff */
	zr->jpg_que_head = 0;
	zr->jpg_dma_head = 0;
	zr->jpg_dma_tail = 0;
	zr->jpg_que_tail = 0;
	zr->jpg_seq_num = 0;
	zr->JPEG_error = 0;
	zr->num_errors = 0;
	zr->jpg_err_seq = 0;
	zr->jpg_err_shift = 0;
	zr->jpg_queued_num = 0;
	for (i = 0; i < BUZ_NUM_STAT_COM; i++)
		zr->stat_com[i] = cpu_to_le32(1);	/* mark as unavailable to zr36057 */
}

static void zr36057_set_jpg(struct zoran *zr, enum zoran_codec_mode mode)
{
	const struct tvnorm *tvn;
	u32 reg;

	tvn = zr->timing;

	pr_info("%s mode=%d %s\n", __func__, mode, codec_name(mode));
	/* assert P_Reset, disable code transfer, deassert Active */
	btwrite(0, ZR36057_JPC);

	/* MJPEG compression mode */
	switch (mode) {
	case BUZ_MODE_MOTION_COMPRESS:
	default:
		reg = ZR36057_JMC_MJPGCmpMode;
		break;

	case BUZ_MODE_MOTION_DECOMPRESS:
		reg = ZR36057_JMC_MJPGExpMode;
		reg |= ZR36057_JMC_SyncMstr;
		/* RJ: The following is experimental - improves the output to screen */
		//if(zr->jpg_settings.VFIFO_FB) reg |= ZR36057_JMC_VFIFO_FB; // No, it doesn't. SM
		break;

	case BUZ_MODE_STILL_COMPRESS:
		reg = ZR36057_JMC_JPGCmpMode;
		break;

	case BUZ_MODE_STILL_DECOMPRESS:
		reg = ZR36057_JMC_JPGExpMode;
		break;
	}
	reg |= ZR36057_JMC_JPG;
	if (zr->jpg_settings.field_per_buff == 1)
		reg |= ZR36057_JMC_Fld_per_buff;
	btwrite(reg, ZR36057_JMC);

	/* vertical */
	btor(ZR36057_VFEVCR_VSPol, ZR36057_VFEVCR);
	reg = (6 << ZR36057_VSP_VsyncSize) |
	      (tvn->Ht << ZR36057_VSP_FrmTot);
	btwrite(reg, ZR36057_VSP);
	reg = ((zr->jpg_settings.img_y + tvn->VStart) << ZR36057_FVAP_NAY) |
	      (zr->jpg_settings.img_height << ZR36057_FVAP_PAY);
	btwrite(reg, ZR36057_FVAP);

	/* horizontal */
	if (zr->card.vfe_pol.hsync_pol) {
		btor(ZR36057_VFEHCR_HSPol, ZR36057_VFEHCR);
	} else {
		btand(~ZR36057_VFEHCR_HSPol, ZR36057_VFEHCR);}
	reg = ((tvn->HSyncStart) << ZR36057_HSP_HsyncStart) |
	      (tvn->Wt << ZR36057_HSP_LineTot);
	btwrite(reg, ZR36057_HSP);
	reg = ((zr->jpg_settings.img_x +
		tvn->HStart + 4) << ZR36057_FHAP_NAX) |
	      (zr->jpg_settings.img_width << ZR36057_FHAP_PAX);
	btwrite(reg, ZR36057_FHAP);

	/* field process parameters */
	if (zr->jpg_settings.odd_even)
		reg = ZR36057_FPP_Odd_Even;
	else
		reg = 0;

	btwrite(reg, ZR36057_FPP);

	/* Set proper VCLK Polarity, else colors will be wrong during playback */
	//btor(ZR36057_VFESPFR_VCLKPol, ZR36057_VFESPFR);

	/* code base address */
	btwrite(zr->p_sc, ZR36057_JCBA);

	/* FIFO threshold (FIFO is 160. double words) */
	/* NOTE: decimal values here */
	switch (mode) {
	case BUZ_MODE_STILL_COMPRESS:
	case BUZ_MODE_MOTION_COMPRESS:
		if (zr->card.type != BUZ)
			reg = 140;
		else
			reg = 60;
		break;

	case BUZ_MODE_STILL_DECOMPRESS:
	case BUZ_MODE_MOTION_DECOMPRESS:
		reg = 20;
		break;

	default:
		reg = 80;
		break;
	}
	btwrite(reg, ZR36057_JCFT);
	zr36057_adjust_vfe(zr, mode);
}

void print_interrupts(struct zoran *zr)
{
	int res, noerr = 0;

	pr_info("%s: interrupts received:", ZR_DEVNAME(zr));
	res = zr->field_counter;
	if (res < -1 || res > 1)
		printk(KERN_CONT " FD:%d", res);
	res = zr->intr_counter_GIRQ1;
	if (res != 0) {
		printk(KERN_CONT " GIRQ1:%d", res);
		noerr++;
	}
	res = zr->intr_counter_GIRQ0;
	if (res != 0) {
		printk(KERN_CONT " GIRQ0:%d", res);
		noerr++;
	}
	if ((res = zr->intr_counter_CodRepIRQ) != 0) {
		printk(KERN_CONT " CodRepIRQ:%d", res);
		noerr++;
	}
	if ((res = zr->intr_counter_JPEGRepIRQ) != 0) {
		printk(KERN_CONT " JPEGRepIRQ:%d", res);
		noerr++;
	}
	if (zr->JPEG_max_missed) {
		printk(KERN_CONT " JPEG delays: max=%d min=%d", zr->JPEG_max_missed,
		       zr->JPEG_min_missed);
	}
	if (zr->END_event_missed) {
		printk(KERN_CONT " ENDs missed: %d", zr->END_event_missed);
	}
	if (!noerr)
		printk(KERN_CONT ": no interrupts detected.");
	printk(KERN_CONT "\n");
}

void clear_interrupt_counters(struct zoran *zr)
{
	zr->intr_counter_GIRQ1 = 0;
	zr->intr_counter_GIRQ0 = 0;
	zr->intr_counter_CodRepIRQ = 0;
	zr->intr_counter_JPEGRepIRQ = 0;
	zr->field_counter = 0;
	zr->IRQ1_in = 0;
	zr->IRQ1_out = 0;
	zr->JPEG_in = 0;
	zr->JPEG_out = 0;
	zr->JPEG_0 = 0;
	zr->JPEG_1 = 0;
	zr->END_event_missed = 0;
	zr->JPEG_missed = 0;
	zr->JPEG_max_missed = 0;
	zr->JPEG_min_missed = 0x7fffffff;
}

static u32 count_reset_interrupt(struct zoran *zr)
{
	u32 isr;

	/*isr = btread(ZR36057_ISR) & 0x78000000;*/
	isr = btread(ZR36057_ISR);
	if (isr) {
		pr_info("%s: %x ICR=%x\n", __func__, isr, btread(ZR36057_ICR));
		if (isr & ZR36057_ISR_GIRQ1) {
			btwrite(ZR36057_ISR_GIRQ1, ZR36057_ISR);
			zr->intr_counter_GIRQ1++;
		}
		if (isr & ZR36057_ISR_GIRQ0) {
			btwrite(ZR36057_ISR_GIRQ0, ZR36057_ISR);
			zr->intr_counter_GIRQ0++;
		}
		if (isr & ZR36057_ISR_CodRepIRQ) {
			btwrite(ZR36057_ISR_CodRepIRQ, ZR36057_ISR);
			zr->intr_counter_CodRepIRQ++;
		}
		if (isr & ZR36057_ISR_JPEGRepIRQ) {
			btwrite(ZR36057_ISR_JPEGRepIRQ, ZR36057_ISR);
			zr->intr_counter_JPEGRepIRQ++;
		}
	}
	return isr;
}

void jpeg_start(struct zoran *zr)
{
	int reg;

	pci_info(zr->pci_dev, "%s\n", __func__);
	zr->frame_num = 0;

	/* deassert P_reset, disable code transfer, deassert Active */
	btwrite(ZR36057_JPC_P_Reset, ZR36057_JPC);
	/* stop flushing the internal code buffer */
	btand(~ZR36057_MCTCR_CFlush, ZR36057_MCTCR);
	/* enable code transfer */
	btor(ZR36057_JPC_CodTrnsEn, ZR36057_JPC);

	/* clear IRQs */
	btwrite(IRQ_MASK, ZR36057_ISR);
	/* enable the JPEG IRQs */
	btwrite(zr->card.jpeg_int | ZR36057_ICR_JPEGRepIRQ | ZR36057_ICR_IntPinEn,
		ZR36057_ICR);

	set_frame(zr, 0);	// \FRAME

	/* set the JPEG codec guest ID */
	reg = (zr->card.gpcs[1] << ZR36057_JCGI_JPEGuestID) |
	       (0 << ZR36057_JCGI_JPEGuestReg);
	btwrite(reg, ZR36057_JCGI);

	if (zr->card.video_vfe == CODEC_TYPE_ZR36016 &&
	    zr->card.video_codec == CODEC_TYPE_ZR36050) {
		/* Enable processing on the ZR36016 */
		if (zr->vfe)
			zr36016_write(zr->vfe, 0, 1);

		/* load the address of the GO register in the ZR36050 latch */
		post_office_write(zr, 0, 0, 0);
	}

	/* assert Active */
	btor(ZR36057_JPC_Active, ZR36057_JPC);

	/* enable the Go generation */
	btor(ZR36057_JMC_Go_en, ZR36057_JMC);
	udelay(30);

	set_frame(zr, 1);	// /FRAME

	dprintk(3, KERN_DEBUG "%s: jpeg_start\n", ZR_DEVNAME(zr));
	zoran_status(zr);
}

void zr36057_enable_jpg(struct zoran *zr, enum zoran_codec_mode mode)
{
	struct vfe_settings cap;
	int field_size = zr->buffer_size / zr->jpg_settings.field_per_buff;

	pci_info(zr->pci_dev, "%s: mode=%d\n", __func__, mode);

	zr->codec_mode = mode;

	cap.x = zr->jpg_settings.img_x;
	cap.y = zr->jpg_settings.img_y;
	cap.width = zr->jpg_settings.img_width;
	cap.height = zr->jpg_settings.img_height;
	cap.decimation =
	    zr->jpg_settings.HorDcm | (zr->jpg_settings.VerDcm << 8);
	cap.quality = zr->jpg_settings.jpg_comp.quality;

	switch (mode) {
	case BUZ_MODE_MOTION_COMPRESS: {
		struct jpeg_app_marker app;
		struct jpeg_com_marker com;

		pci_info(zr->pci_dev, "%s BUZ_MODE_MOTION_COMPRESS\n", __func__);
		/* In motion compress mode, the decoder output must be enabled, and
		 * the video bus direction set to input.
		 */
		set_videobus_dir(zr, 0);
		pr_info("%s decoder_call s_stream 1\n", __func__);
		decoder_call(zr, video, s_stream, 1);
		pr_info("%s encoder_call s_routing 0 0 0\n", __func__);
		encoder_call(zr, video, s_routing, 0, 0, 0);

		/* Take the JPEG codec and the VFE out of sleep */
		jpeg_codec_sleep(zr, 0);

		/* set JPEG app/com marker */
		app.appn = zr->jpg_settings.jpg_comp.APPn;
		app.len = zr->jpg_settings.jpg_comp.APP_len;
		memcpy(app.data, zr->jpg_settings.jpg_comp.APP_data, 60);
		zr->codec->control(zr->codec, CODEC_S_JPEG_APP_DATA,
				   sizeof(struct jpeg_app_marker), &app);

		com.len = zr->jpg_settings.jpg_comp.COM_len;
		memcpy(com.data, zr->jpg_settings.jpg_comp.COM_data, 60);
		zr->codec->control(zr->codec, CODEC_S_JPEG_COM_DATA,
				   sizeof(struct jpeg_com_marker), &com);

		/* Setup the JPEG codec */
		zr->codec->control(zr->codec, CODEC_S_JPEG_TDS_BYTE,
				   sizeof(int), &field_size);
		zr->codec->set_video(zr->codec, zr->timing, &cap,
				     &zr->card.vfe_pol);
		zr->codec->set_mode(zr->codec, CODEC_DO_COMPRESSION);

		/* Setup the VFE */
		if (zr->vfe) {
			zr->vfe->control(zr->vfe, CODEC_S_JPEG_TDS_BYTE,
					 sizeof(int), &field_size);
			zr->vfe->set_video(zr->vfe, zr->timing, &cap,
					   &zr->card.vfe_pol);
			zr->vfe->set_mode(zr->vfe, CODEC_DO_COMPRESSION);
		}

		init_jpeg_queue(zr);
		zr36057_set_jpg(zr, mode);	// \P_Reset, ... Video param, FIFO

		clear_interrupt_counters(zr);
		dprintk(2, KERN_INFO "%s: enable_jpg(MOTION_COMPRESS)\n",
			ZR_DEVNAME(zr));
		break;
	}

	case BUZ_MODE_MOTION_DECOMPRESS:
		pci_info(zr->pci_dev, "%s BUZ_MODE_MOTION_DECOMPRESS\n", __func__);
		/* In motion decompression mode, the decoder output must be disabled, and
		 * the video bus direction set to output.
		 */
		pr_info("%s decoder_call s_stream 0\n", __func__);
		decoder_call(zr, video, s_stream, 0);
		set_videobus_dir(zr, 1);
		pr_info("%s encoder_call s_routing 1 0 0\n", __func__);
		encoder_call(zr, video, s_routing, 1, 0, 0);

		/* Take the JPEG codec and the VFE out of sleep */
		jpeg_codec_sleep(zr, 0);
		/* Setup the VFE */
		if (zr->vfe) {
			zr->vfe->set_video(zr->vfe, zr->timing, &cap,
					   &zr->card.vfe_pol);
			zr->vfe->set_mode(zr->vfe, CODEC_DO_EXPANSION);
		}
		/* Setup the JPEG codec */
		zr->codec->set_video(zr->codec, zr->timing, &cap,
				     &zr->card.vfe_pol);
		zr->codec->set_mode(zr->codec, CODEC_DO_EXPANSION);

		init_jpeg_queue(zr);
		zr36057_set_jpg(zr, mode);	// \P_Reset, ... Video param, FIFO

		clear_interrupt_counters(zr);
		dprintk(2, KERN_INFO "%s: enable_jpg(MOTION_DECOMPRESS)\n",
			ZR_DEVNAME(zr));
		break;

	default:
		pci_warn(zr->pci_dev, "%s invalid mode\n", __func__ );
	case BUZ_MODE_IDLE:
		pci_info(zr->pci_dev, "%s BUZ_MODE_IDLE\n", __func__);
		/* shut down processing */
		btand(~(zr->card.jpeg_int | ZR36057_ICR_JPEGRepIRQ),
		      ZR36057_ICR);
		btwrite(zr->card.jpeg_int | ZR36057_ICR_JPEGRepIRQ,
			ZR36057_ISR);
		btand(~ZR36057_JMC_Go_en, ZR36057_JMC);	// \Go_en

		msleep(50);

		set_videobus_dir(zr, 0);
		set_frame(zr, 1);	// /FRAME
		btor(ZR36057_MCTCR_CFlush, ZR36057_MCTCR);	// /CFlush
		btwrite(0, ZR36057_JPC);	// \P_Reset,\CodTrnsEn,\Active
		btand(~ZR36057_JMC_VFIFO_FB, ZR36057_JMC);
		btand(~ZR36057_JMC_SyncMstr, ZR36057_JMC);
		jpeg_codec_reset(zr);
		jpeg_codec_sleep(zr, 1);
		zr36057_adjust_vfe(zr, mode);

		pr_info("%s encoder_call s_stream 1\n", __func__);
		decoder_call(zr, video, s_stream, 1);
		pr_info("%s decoder_call s_routing 0 0 0\n", __func__);
		encoder_call(zr, video, s_routing, 0, 0, 0);

		dprintk(2, KERN_INFO "%s: enable_jpg(IDLE)\n", ZR_DEVNAME(zr));
		break;
	}
}

/* when this is called the spinlock must be held */
void zoran_feed_stat_com(struct zoran *zr)
{
	/* move frames from pending queue to DMA */

	int frame, i, max_stat_com;
	struct zr_vout_buffer *buf;
	struct vb2_v4l2_buffer *vbuf;
	dma_addr_t phys_addr = 0;
	unsigned long flags;
	unsigned long payload;

	max_stat_com =
	    (zr->jpg_settings.TmpDcm ==
	     1) ? BUZ_NUM_STAT_COM : (BUZ_NUM_STAT_COM >> 1);

	pci_info(zr->pci_dev, "%s max=%d dma=%ld/%ld\n", __func__, max_stat_com, zr->jpg_dma_head, zr->jpg_dma_tail);

	spin_lock_irqsave(&zr->queued_bufs_lock, flags);
	while ((zr->jpg_dma_head - zr->jpg_dma_tail) < max_stat_com) {
		buf = list_first_entry_or_null(&zr->queued_bufs, struct zr_vout_buffer, queue);
		if (!buf) {
			pci_err(zr->pci_dev, "No buffer availlable to queue\n");
			spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
			return;
		}
		list_del(&buf->queue);
		zr->buf_in_reserve--;
		vbuf = &buf->vbuf;
		vbuf->vb2_buf.state = VB2_BUF_STATE_ACTIVE;
		phys_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);
		payload = vb2_get_plane_payload(&vbuf->vb2_buf, 0);
		if (payload == 0)
			payload = zr->buffer_size;
		if (zr->jpg_settings.TmpDcm == 1) {
			/* fill 1 stat_com entry */
			i = (zr->jpg_dma_head -
			     zr->jpg_err_shift) & BUZ_MASK_STAT_COM;
			if (!(zr->stat_com[i] & cpu_to_le32(1)))
				break;
/*			pr_info("%s Feed stat_com %d buf=%px phy=%llx\n", __func__, i, buf, phys_addr);*/
			zr->stat_comb[i * 2] = cpu_to_le32(phys_addr);
			zr->stat_comb[i * 2 + 1] = cpu_to_le32((payload >> 1)| 1);
			zr->inuse[i] = buf;
			zr->stat_com[i] = zr->p_scb + i * 2 * 4;
/*			pr_info("%s stat_com %d point to scb %px+%d %px buf=%px remains=%d\n",
				__func__, i, zr->p_scb, i * 2, zr->p_scb + i * 2 * 4, vbuf, zr->buf_in_reserve);*/
		} else {
			/* fill 2 stat_com entries */
			i = ((zr->jpg_dma_head -
			      zr->jpg_err_shift) & 1) * 2;
			if (!(zr->stat_com[i] & cpu_to_le32(1)))
				break;
			/*pr_info("%s Feed stat_com %d x2 %px\n", __func__, i, buf);*/
			zr->stat_com[i] = zr->p_scb + i * 2 * 4;
			zr->stat_com[i + 1] = zr->p_scb + i * 2 * 4;

			zr->stat_comb[i * 2] = cpu_to_le32(phys_addr);
			zr->stat_comb[i * 2 + 1] = cpu_to_le32((payload >> 1)| 1);

			zr->inuse[i] = buf;
			zr->inuse[i + 1] = 0;
		}
		zr->jpg_dma_head++;
	}
	spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
	if (zr->codec_mode == BUZ_MODE_MOTION_DECOMPRESS)
		zr->jpg_queued_num++;
}

static int first;

/* when this is called the spinlock must be held */
static void zoran_reap_stat_com(struct zoran *zr)
{
	/* move frames from DMA queue to done queue */

	int i;
	u32 stat_com;
	unsigned int seq;
	unsigned int dif;
	unsigned long flags;
	int frame;
	struct zr_vout_buffer *buf;
	unsigned int size = 0;
	u32 fcnt;

	/* In motion decompress we don't have a hardware frame counter,
	 * we just count the interrupts here */

	if (zr->codec_mode == BUZ_MODE_MOTION_DECOMPRESS)
		zr->jpg_seq_num++;

	spin_lock_irqsave(&zr->queued_bufs_lock, flags);
	while (zr->jpg_dma_tail < zr->jpg_dma_head) {
		if (zr->jpg_settings.TmpDcm == 1)
			i = (zr->jpg_dma_tail -
			     zr->jpg_err_shift) & BUZ_MASK_STAT_COM;
		else
			i = ((zr->jpg_dma_tail -
			      zr->jpg_err_shift) & 1) * 2 + 1;

		stat_com = le32_to_cpu(zr->stat_com[i]);
		if ((stat_com & 1) == 0) {
			spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
			return;
		}

		fcnt = (stat_com & GENMASK(31,24)) >> 24;
		size = (stat_com & GENMASK(22,1)) >> 1;

		buf = zr->inuse[i];
		buf->vbuf.vb2_buf.timestamp = ktime_get_ns();

		if (zr->codec_mode == BUZ_MODE_MOTION_COMPRESS) {
			/*size = (stat_com & 0x7fffff) >> 1;*/
			vb2_set_plane_payload(&buf->vbuf.vb2_buf, 0, size);

			/* update sequence number with the help of the counter in stat_com */
			seq = ((stat_com >> 24) + zr->jpg_err_seq) & 0xff;
			dif = (seq - zr->jpg_seq_num) & 0xff;
			zr->jpg_seq_num += dif;
		}
		buf->vbuf.sequence =
		    zr->jpg_settings.TmpDcm ==
		    2 ? (zr->jpg_seq_num >> 1) : zr->jpg_seq_num;
		zr->inuse[i] = NULL;
		if (zr->jpg_settings.TmpDcm != 1)
			buf->vbuf.field = zr->jpg_settings.odd_even ?
				V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM;
		else
			buf->vbuf.field = zr->jpg_settings.odd_even ?
				V4L2_FIELD_SEQ_TB : V4L2_FIELD_SEQ_BT;
		pci_info(zr->pci_dev, "Finish buffer %d seq=%d size=%d field=%x bufaddr=%px\n",
			i, buf->vbuf.sequence, size, buf->vbuf.field, buf);
		vb2_buffer_done(&buf->vbuf.vb2_buf, VB2_BUF_STATE_DONE);

		zr->jpg_dma_tail++;
		if (!first) {
			zoran_status(zr);
			first = 1;
		}
	}
	spin_unlock_irqrestore(&zr->queued_bufs_lock, flags);
}

static void zoran_restart(struct zoran *zr)
{
	/* Now the stat_comm buffer is ready for restart */
	unsigned int status = 0;
	int mode;

	pci_info(zr->pci_dev, "%s\n", __func__);
	if (zr->codec_mode == BUZ_MODE_MOTION_COMPRESS) {
		decoder_call(zr, video, g_input_status, &status);
		pr_info("%s decoder_call video g_input_status status=%d\n", __func__, status);
		mode = CODEC_DO_COMPRESSION;
	} else {
		status = V4L2_IN_ST_NO_SIGNAL;
		mode = CODEC_DO_EXPANSION;
	}
	if (zr->codec_mode == BUZ_MODE_MOTION_DECOMPRESS ||
	    !(status & V4L2_IN_ST_NO_SIGNAL)) {
		/********** RESTART code *************/
		jpeg_codec_reset(zr);
		zr->codec->set_mode(zr->codec, mode);
		zr36057_set_jpg(zr, zr->codec_mode);
		jpeg_start(zr);

		if (zr->num_errors <= 8)
			dprintk(2, KERN_INFO "%s: Restart\n",
				ZR_DEVNAME(zr));

		zr->JPEG_missed = 0;
		zr->JPEG_error = 2;
		/********** End RESTART code ***********/
	}

	zr->running = ZORAN_MAP_MODE_NONE;
}

static void error_handler(struct zoran *zr, u32 astat, u32 stat)
{
	int i;

	pr_info("%s XXXXX\n", __func__);
}

irqreturn_t zoran_irqng(int irq, void *dev_id) {
	struct zoran *zr = dev_id;
	u32 stat, astat;

	stat = count_reset_interrupt(zr);
	astat = stat & IRQ_MASK;
	if (astat & zr->card.vsync_int) {
		pr_info("VSYNC IRQ memgrab=%d\n", zr->v4l_memgrab_active);
		if (zr->v4l_memgrab_active) {
			if ((btread(ZR36057_VSSFGR) & ZR36057_VSSFGR_SnapShot) == 0)
				pr_warn("%s: BuzIRQ with SnapShot off ???\n", ZR_DEVNAME(zr));
			if ((btread(ZR36057_VSSFGR) & ZR36057_VSSFGR_FrameGrab) == 0) {
				zr_set_buf(zr);
			} else {
				pr_info("%s: WAIT FOR END\n", __func__);
			}
			return IRQ_HANDLED;
		}
		if (astat & ZR36057_ISR_JPEGRepIRQ) {
			if (zr->codec_mode != BUZ_MODE_MOTION_DECOMPRESS &&
				zr->codec_mode != BUZ_MODE_MOTION_COMPRESS) {
				pci_err(zr->pci_dev, "JPG IRQ when not in good mode\n");
				return IRQ_HANDLED;
			}
		zr->frame_num++;
		zoran_reap_stat_com(zr);
		zoran_feed_stat_com(zr);
		return IRQ_HANDLED;
		}
		/* unused interupts */
	}
	zr->ghost_int++;
	return IRQ_HANDLED;
}

irqreturn_t zoran_irq(int irq, void *dev_id)
{
	u32 stat, astat;
	int count = 0;
	struct zoran *zr = dev_id;
	unsigned long flags;
	int loop = 0;

	if (zr->testing) {
		pr_info("%s: testing\n", __func__);
		/* Testing interrupts */
		spin_lock_irqsave(&zr->spinlock, flags);
		while ((stat = count_reset_interrupt(zr))) {
			if (count++ > 100) {
				btand(~ZR36057_ICR_IntPinEn, ZR36057_ICR);
				pr_err("%s: IRQ lockup while testing, isr=0x%08x, cleared int mask\n",
				       ZR_DEVNAME(zr), stat);
				wake_up_interruptible(&zr->test_q);
			}
		}
		zr->last_isr = stat;
		spin_unlock_irqrestore(&zr->spinlock, flags);
		return IRQ_HANDLED;
	}

/*	spin_lock_irqsave(&zr->spinlock, flags);*/
	while (loop < 10) {
		/* get/clear interrupt status bits */
		stat = count_reset_interrupt(zr);
		astat = stat & IRQ_MASK;
/*		if (loop++ < 10)
			pr_info("%s: astat: 0x%08x, mask: 0x%08x loop=%d stat=%x\n",
				__func__, astat, btread(ZR36057_ICR), loop, stat);*/
		if (!astat)
			break;
		if (astat & zr->card.vsync_int) {	// SW
			if (zr->v4l_memgrab_active) {
				if ((btread(ZR36057_VSSFGR) & ZR36057_VSSFGR_FrameGrab) == 0) {
					zr_set_buf(zr);
				} else {
					/*pr_info("%s: WAIT FOR END\n", __func__);*/
				}
				return IRQ_HANDLED;
			}
			if (zr->codec_mode == BUZ_MODE_MOTION_DECOMPRESS ||
			    zr->codec_mode == BUZ_MODE_MOTION_COMPRESS) {
				/* count missed interrupts */
				zr->JPEG_missed++;
			}
			//post_office_read(zr,1,0);
			/*
			 * Interrupts may still happen when
			 * zr->v4l_memgrab_active is switched off.
			 * We simply ignore them
			 */

			if (zr->v4l_memgrab_active) {
				/* A lot more checks should be here ... */
				if ((btread(ZR36057_VSSFGR) & ZR36057_VSSFGR_SnapShot) == 0)
					pr_warn("%s: BuzIRQ with SnapShot off ???\n",
						ZR_DEVNAME(zr));

				if (zr->v4l_grab_frame != NO_GRAB_ACTIVE) {
					/* There is a grab on a frame going on, check if it has finished */
					if ((btread(ZR36057_VSSFGR) & ZR36057_VSSFGR_FrameGrab) == 0) {
						/* it is finished, notify the user */
					}
				}
			}

			/*
			 * even if we don't grab, we do want to increment
			 * the sequence counter to see lost frames
			 */
			zr->v4l_grab_seq++;
		}
#if (IRQ_MASK & ZR36057_ISR_CodRepIRQ)
		if (astat & ZR36057_ISR_CodRepIRQ) {
			zr->intr_counter_CodRepIRQ++;
			pr_info("%s: ZR36057_ISR_CodRepIRQ\n", ZR_DEVNAME(zr));
			btand(~ZR36057_ICR_CodRepIRQ, ZR36057_ICR);
		}
#endif				/* (IRQ_MASK & ZR36057_ISR_CodRepIRQ) */

#if (IRQ_MASK & ZR36057_ISR_JPEGRepIRQ)
		if ((astat & ZR36057_ISR_JPEGRepIRQ) &&
		    (zr->codec_mode == BUZ_MODE_MOTION_DECOMPRESS ||
		     zr->codec_mode == BUZ_MODE_MOTION_COMPRESS)) {
			if (zr36067_debug > 1 && (!zr->frame_num || zr->JPEG_error)) {
				char sv[BUZ_NUM_STAT_COM + 1];
				int i;

				pr_info("%s: first frame ready: state=0x%08x odd_even=%d field_per_buff=%d delay=%d\n",
					ZR_DEVNAME(zr), stat, zr->jpg_settings.odd_even,
				       zr->jpg_settings.field_per_buff, zr->JPEG_missed);

				for (i = 0; i < BUZ_NUM_STAT_COM; i++)
					sv[i] = le32_to_cpu(zr->stat_com[i]) & 1 ? '1' : '0';
				sv[BUZ_NUM_STAT_COM] = 0;
				pr_info("%s: stat_com=%s queue_state=%ld/%ld/%ld/%ld\n",
					ZR_DEVNAME(zr), sv, zr->jpg_que_tail, zr->jpg_dma_tail,
				       zr->jpg_dma_head, zr->jpg_que_head);
			} else {
				/* Get statistics */
				if (zr->JPEG_missed > zr->JPEG_max_missed)
					zr->JPEG_max_missed = zr->JPEG_missed;
				if (zr->JPEG_missed < zr->JPEG_min_missed)
					zr->JPEG_min_missed = zr->JPEG_missed;
			}

			if (zr36067_debug > 2 && zr->frame_num < 6) {
				int i;

				pr_info("%s: seq=%ld stat_com:", ZR_DEVNAME(zr), zr->jpg_seq_num);
				for (i = 0; i < 4; i++)
					printk(KERN_CONT " %08x", le32_to_cpu(zr->stat_com[i]));
				printk(KERN_CONT "\n");
			}
			zr->frame_num++;
			zr->JPEG_missed = 0;
			zr->JPEG_error = 0;
			zoran_reap_stat_com(zr);
			/*zoran_feed_stat_com(zr);*/
			wake_up_interruptible(&zr->jpg_capq);
		}
#endif				/* (IRQ_MASK & ZR36057_ISR_JPEGRepIRQ) */

		/* DATERR, too many fields missed, error processing */
		if ((astat & zr->card.jpeg_int) ||
		    zr->JPEG_missed > 25 ||
		    zr->JPEG_error == 1	||
		    ((zr->codec_mode == BUZ_MODE_MOTION_DECOMPRESS) &&
		     (zr->frame_num && (zr->JPEG_missed > zr->jpg_settings.field_per_buff)))) {
			error_handler(zr, astat, stat);
		}

		count++;
		if (count > 10) {
			pr_warn("%s: irq loop %d\n", ZR_DEVNAME(zr), count);
			if (count > 20) {
				btand(~ZR36057_ICR_IntPinEn, ZR36057_ICR);
				pr_err("%s: IRQ lockup, cleared int mask\n", ZR_DEVNAME(zr));
				break;
			}
		}
		zr->last_isr = stat;
	}
/*	spin_unlock_irqrestore(&zr->spinlock, flags);*/

	return IRQ_HANDLED;
}

void zoran_set_pci_master(struct zoran *zr, int set_master)
{
	pci_info(zr->pci_dev, "%s setmaster=%d\n", __func__, set_master);
	if (set_master) {
		pci_set_master(zr->pci_dev);
	} else {
		u16 command;

		pci_read_config_word(zr->pci_dev, PCI_COMMAND, &command);
		command &= ~PCI_COMMAND_MASTER;
		pci_write_config_word(zr->pci_dev, PCI_COMMAND, command);
	}
}

void zoran_init_hardware(struct zoran *zr)
{

	pci_info(zr->pci_dev,"%s %d\n", __func__, __LINE__);
	/* Enable bus-mastering */
	zoran_set_pci_master(zr, 1);

	/* Initialize the board */
	if (zr->card.init)
		zr->card.init(zr);

	pr_info("%s decoder_call core init 0\n", __func__);
	decoder_call(zr, core, init, 0);
	pr_info("%s decoder_call s_std norm=%lld\n", __func__, zr->norm);
	decoder_call(zr, video, s_std, zr->norm);
	pr_info("%s decoder_call s_routing muxsel=%d 0 0\n", __func__, zr->card.input[zr->input].muxsel);
	decoder_call(zr, video, s_routing,
		     zr->card.input[zr->input].muxsel, 0, 0);

	pr_info("%s encoder_call core init 0\n", __func__);
	encoder_call(zr, core, init, 0);
	pr_info("%s encoder_call s_std_output norm=%lld\n", __func__, zr->norm);
	encoder_call(zr, video, s_std_output, zr->norm);
	pr_info("%s encoder_call s_routing 0 0 0\n", __func__);
	encoder_call(zr, video, s_routing, 0, 0, 0);

	/* toggle JPEG codec sleep to sync PLL */
	jpeg_codec_sleep(zr, 1);
	jpeg_codec_sleep(zr, 0);

	/*
	 * set individual interrupt enables (without GIRQ1)
	 * but don't global enable until zoran_open()
	 */
	zr36057_init_vfe(zr);

	zr36057_enable_jpg(zr, BUZ_MODE_IDLE);

	btwrite(IRQ_MASK, ZR36057_ISR);
}

void zr36057_restart(struct zoran *zr)
{
	pr_info("%s\n", __func__);
	btwrite(0, ZR36057_SPGPPCR);
	udelay(1000);
	btor(ZR36057_SPGPPCR_SoftReset, ZR36057_SPGPPCR);
	udelay(1000);

	/* assert P_Reset */
	btwrite(0, ZR36057_JPC);
	/* set up GPIO direction - all output */
	btwrite(ZR36057_SPGPPCR_SoftReset | 0, ZR36057_SPGPPCR);

	/* set up GPIO pins and guest bus timing */
	btwrite((0x81 << 24) | 0x8888, ZR36057_GPPGCR1);
}

