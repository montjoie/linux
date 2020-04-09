/*
 * zoran - Iomega Buz driver
 *
 * Copyright (C) 1999 Rainer Johanni <Rainer@Johanni.de>
 *
 * based on
 *
 * zoran.0.0.3 Copyright (C) 1998 Dave Perks <dperks@ibm.net>
 *
 * and
 *
 * bttv - Bt848 frame grabber driver
 * Copyright (C) 1996,97,98 Ralph  Metzler (rjkm@thp.uni-koeln.de)
 *                        & Marcus Metzler (mocm@thp.uni-koeln.de)
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

#ifndef _BUZ_H_
#define _BUZ_H_

#include <linux/debugfs.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

/* #define ZORAN_OLD */
/*#define COMPLIANCE*/

#define ZR_NORM_PAL 0
#define ZR_NORM_NTSC 1
#define ZR_NORM_SECAM 2

static const char *norm_name(int c)
{
	switch(c) {
	case ZR_NORM_PAL:
		return "PAL";
	case ZR_NORM_NTSC:
		return "NTSC";
	case ZR_NORM_SECAM:
		return "SECAM";
	default:
		if (c & V4L2_STD_SECAM)
			return "SECAM";
		else if (c & V4L2_STD_NTSC)
			return "NTSC";
		else
			return "PAL";
	}
	return "UNKNOW NORM";
}

struct zoran_sync {
	unsigned long frame;	/* number of buffer that has been free'd */
	unsigned long length;	/* number of code bytes in buffer (capture only) */
	unsigned long seq;	/* frame sequence number */
	u64 ts;			/* timestamp */
};

struct zr_vout_buffer {
       /* common v4l buffer stuff -- must be first */
       struct vb2_v4l2_buffer          vbuf;
       struct list_head                queue;
};

static inline struct zr_vout_buffer *vb2_to_zr_vout_buffer(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	return container_of(vbuf, struct zr_vout_buffer, vbuf);
}

#define ZORAN_NAME    "ZORAN"	/* name of the device */

#define ZR_DEVNAME(zr) ((zr)->name)

#define   BUZ_MAX_WIDTH   (zr->timing->Wa)
#define   BUZ_MAX_HEIGHT  (zr->timing->Ha)
#define   BUZ_MIN_WIDTH    32	/* never display less than 32 pixels */
#define   BUZ_MIN_HEIGHT   24	/* never display less than 24 rows */

#define BUZ_NUM_STAT_COM    4
#define BUZ_MASK_STAT_COM   3

#define BUZ_MAX_FRAME     256	/* Must be a power of 2 */
#define BUZ_MASK_FRAME    255	/* Must be BUZ_MAX_FRAME-1 */

#define BUZ_MAX_INPUT       16

#if VIDEO_MAX_FRAME <= 32
#   define   V4L_MAX_FRAME   32
#elif VIDEO_MAX_FRAME <= 64
#   define   V4L_MAX_FRAME   64
#else
#   error   "Too many video frame buffers to handle"
#endif
#define   V4L_MASK_FRAME   (V4L_MAX_FRAME - 1)

#define MAX_FRAME (BUZ_MAX_FRAME > VIDEO_MAX_FRAME ? BUZ_MAX_FRAME : VIDEO_MAX_FRAME)

#include "zr36057.h"

enum card_type {
	UNKNOWN = -1,

	/* Pinnacle/Miro */
	DC10_old,		/* DC30 like */
	DC10_new,		/* DC10plus like */
	DC10plus,
	DC30,
	DC30plus,

	/* Linux Media Labs */
	LML33,
	LML33R10,

	/* Iomega */
	BUZ,

	/* AverMedia */
	AVS6EYES,

	/* total number of cards */
	NUM_CARDS
};

enum zoran_codec_mode {
	BUZ_MODE_IDLE,		/* nothing going on */
	BUZ_MODE_MOTION_COMPRESS,	/* grabbing frames */
	BUZ_MODE_MOTION_DECOMPRESS,	/* playing frames */
	BUZ_MODE_STILL_COMPRESS,	/* still frame conversion */
	BUZ_MODE_STILL_DECOMPRESS	/* still frame conversion */
};

static const char *codec_name(int c)
{
	switch(c) {
	case BUZ_MODE_IDLE:
		return "BUZ_MODE_IDLE";
	case BUZ_MODE_MOTION_COMPRESS:
		return "BUZ_MODE_MOTION_COMPRESS";
	case BUZ_MODE_MOTION_DECOMPRESS:
		return "BUZ_MODE_MOTION_DECOMPRESS";
	}
	return "UNKNOW CODE";
}

#ifdef ZORAN_OLD
enum zoran_buffer_state {
	BUZ_STATE_USER,		/* buffer is owned by application */
	BUZ_STATE_PEND,		/* buffer is queued in pend[] ready to feed to I/O */
	BUZ_STATE_DMA,		/* buffer is queued in dma[] for I/O */
	BUZ_STATE_DONE		/* buffer is ready to return to application */
};
#endif
enum zoran_map_mode {
	ZORAN_MAP_MODE_RAW,
	ZORAN_MAP_MODE_JPG_REC,
#define ZORAN_MAP_MODE_JPG ZORAN_MAP_MODE_JPG_REC
	ZORAN_MAP_MODE_JPG_PLAY,
};

static const char *map_mode(int c)
{
	switch(c) {
	case ZORAN_MAP_MODE_RAW:
		return "ZORAN_MAP_MODE_RAW";
	case ZORAN_MAP_MODE_JPG_REC:
		return "ZORAN_MAP_MODE_JPG_REC";
	case ZORAN_MAP_MODE_JPG_PLAY:
		return "ZORAN_MAP_MODE_JPG_PLAY";
	}
	return "UNKNOWN";
}

enum gpio_type {
	ZR_GPIO_JPEG_SLEEP = 0,
	ZR_GPIO_JPEG_RESET,
	ZR_GPIO_JPEG_FRAME,
	ZR_GPIO_VID_DIR,
	ZR_GPIO_VID_EN,
	ZR_GPIO_VID_RESET,
	ZR_GPIO_CLK_SEL1,
	ZR_GPIO_CLK_SEL2,
	ZR_GPIO_MAX,
};

enum gpcs_type {
	GPCS_JPEG_RESET = 0,
	GPCS_JPEG_START,
	GPCS_MAX,
};

struct zoran_format {
	char *name;
	__u32 fourcc;
	int colorspace;
	int depth;
	__u32 flags;
	__u32 vfespfr;
};

/* flags */
#define ZORAN_FORMAT_COMPRESSED BIT(0)
#define ZORAN_FORMAT_OVERLAY BIT(1)
#define ZORAN_FORMAT_CAPTURE BIT(2)
#define ZORAN_FORMAT_PLAYBACK BIT(3)

/* v4l-capture settings */
struct zoran_v4l_settings {
	int width, height, bytesperline;	/* capture size */
	const struct zoran_format *format;	/* capture format */
};

/* jpg-capture/-playback settings */
struct zoran_jpg_settings {
	int decimation;		/* this bit is used to set everything to default */
	int HorDcm, VerDcm, TmpDcm;	/* capture decimation settings (TmpDcm=1 means both fields) */
	int field_per_buff, odd_even;	/* field-settings (odd_even=1 (+TmpDcm=1) means top-field-first) */
	int img_x, img_y, img_width, img_height;	/* crop settings (subframe capture) */
	struct v4l2_jpegcompression jpg_comp;	/* JPEG-specific capture settings */
};

#ifdef ZORAN_OLD
struct zoran_fh;
#endif

#ifdef ZORAN_OLD
struct zoran_mapping {
	struct zoran_fh *fh;
	atomic_t count;
};

struct zoran_buffer {
	struct zoran_mapping *map;
	enum zoran_buffer_state state;	/* state: unused/pending/dma/done */
	struct zoran_sync bs;		/* DONE: info to return to application */
	union {
		struct {
			__le32 *frag_tab;	/* addresses of frag table */
			u32 frag_tab_bus;	/* same value cached to save time in ISR */
		} jpg;
		struct {
			char *fbuffer;		/* virtual address of frame buffer */
			unsigned long fbuffer_phys;/* physical address of frame buffer */
			unsigned long fbuffer_bus;/* bus address of frame buffer */
		} v4l;
	};
};

enum zoran_lock_activity {
	ZORAN_FREE,		/* free for use */
	ZORAN_ACTIVE,		/* active but unlocked */
	ZORAN_LOCKED,		/* locked */
};

/* buffer collections */
struct zoran_buffer_col {
	enum zoran_lock_activity active;	/* feature currently in use? */
	unsigned int num_buffers;
	struct zoran_buffer buffer[MAX_FRAME];	/* buffers */
	u8 allocated;		/* Flag if buffers are allocated */
	u8 need_contiguous;	/* Flag if contiguous buffers are needed */
	/* only applies to jpg buffers, raw buffers are always contiguous */
};
#endif

struct zoran;

/* zoran_fh contains per-open() settings */
struct zoran_fh {
	struct v4l2_fh fh;
	struct zoran *zr;

#ifdef ZORAN_OLD
	enum zoran_map_mode map_mode;		/* Flag which bufferset will map by next mmap() */

	struct zoran_buffer_col buffers;	/* buffers' info */
#endif
};

struct card_info {
	enum card_type type;
	char name[32];
	const char *i2c_decoder;	/* i2c decoder device */
	const unsigned short *addrs_decoder;
	const char *i2c_encoder;	/* i2c encoder device */
	const unsigned short *addrs_encoder;
	u16 video_vfe, video_codec;			/* videocodec types */
	u16 audio_chip;					/* audio type */

	int inputs;		/* number of video inputs */
	struct input {
		int muxsel;
		char name[32];
	} input[BUZ_MAX_INPUT];

	v4l2_std_id norms;
	struct tvnorm *tvn[3];	/* supported TV norms */

	u32 jpeg_int;		/* JPEG interrupt */
	u32 vsync_int;		/* VSYNC interrupt */
	s8 gpio[ZR_GPIO_MAX];
	u8 gpcs[GPCS_MAX];

	struct vfe_polarity vfe_pol;
	u8 gpio_pol[ZR_GPIO_MAX];

	/* is the /GWS line connected? */
	u8 gws_not_connected;

	/* avs6eyes mux setting */
	u8 input_mux;

	void (*init)(struct zoran *zr);
};

struct zoran {
	struct v4l2_device v4l2_dev;
	struct v4l2_ctrl_handler hdl;
	struct video_device *video_dev;
	struct vb2_queue vq;

	struct i2c_adapter i2c_adapter;	/* */
	struct i2c_algo_bit_data i2c_algo;	/* */
	u32 i2cbr;

	struct v4l2_subdev *decoder;	/* video decoder sub-device */
	struct v4l2_subdev *encoder;	/* video encoder sub-device */

	struct videocodec *codec;	/* video codec */
	struct videocodec *vfe;	/* video front end */

	struct mutex lock;	/* file ops serialize lock */

	u8 initialized;		/* flag if zoran has been correctly initialized */
#ifdef ZORAN_OLD
	int user;		/* number of current users */
#endif
	struct card_info card;
	struct tvnorm *timing;

	unsigned short id;	/* number of this device */
	char name[32];		/* name of this device */
	struct pci_dev *pci_dev;	/* PCI device */
	unsigned char revision;	/* revision of zr36057 */
	unsigned char __iomem *zr36057_mem;/* pointer to mapped IO memory */

	spinlock_t spinlock;	/* Spinlock */

	/* Video for Linux parameters */
	int input;	/* card's norm and input */
	v4l2_std_id norm;

	/* Current buffer params */
	unsigned int buffer_size;
	void *vbuf_base;
	int vbuf_height, vbuf_width;
	int vbuf_depth;
	int vbuf_bytesperline;

	wait_queue_head_t v4l_capq;

	int v4l_memgrab_active;	/* Memory grab is activated */

	int v4l_grab_frame;	/* Frame number being currently grabbed */
#define NO_GRAB_ACTIVE (-1)
	unsigned long v4l_grab_seq;	/* Number of frames grabbed */
	struct zoran_v4l_settings v4l_settings;	/* structure with a lot of things to play with */

#ifdef ZORAN_OLD
	/* V4L grab queue of frames pending */
	unsigned long v4l_pend_head;
	unsigned long v4l_pend_tail;
	unsigned long v4l_sync_tail;
	int v4l_pend[V4L_MAX_FRAME];
	struct zoran_buffer_col v4l_buffers;	/* V4L buffers' info */
#endif
	/* Buz MJPEG parameters */
	enum zoran_codec_mode codec_mode;	/* status of codec */
	struct zoran_jpg_settings jpg_settings;	/* structure with a lot of things to play with */

	wait_queue_head_t jpg_capq;	/* wait here for grab to finish */

	/* grab queue counts/indices, mask with BUZ_MASK_STAT_COM before using as index */
	/* (dma_head - dma_tail) is number active in DMA, must be <= BUZ_NUM_STAT_COM */
	/* (value & BUZ_MASK_STAT_COM) corresponds to index in stat_com table */
	unsigned long jpg_que_head;	/* Index where to put next buffer which is queued */
	unsigned long jpg_dma_head;	/* Index of next buffer which goes into stat_com */
	unsigned long jpg_dma_tail;	/* Index of last buffer in stat_com */
	unsigned long jpg_que_tail;	/* Index of last buffer in queue */
	unsigned long jpg_seq_num;	/* count of frames since grab/play started */
	unsigned long jpg_err_seq;	/* last seq_num before error */
	unsigned long jpg_err_shift;
	unsigned long jpg_queued_num;	/* count of frames queued since grab/play started */

	/* zr36057's code buffer table */
	__le32 *stat_com;		/* stat_com[i] is indexed by dma_head/tail & BUZ_MASK_STAT_COM */

	/* (value & BUZ_MASK_FRAME) corresponds to index in pend[] queue */
	int jpg_pend[BUZ_MAX_FRAME];

#ifdef ZORAN_OLD
	/* array indexed by frame number */
	struct zoran_buffer_col jpg_buffers;	/* MJPEG buffers' info */
#endif

	/* Additional stuff for testing */
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *zoran_proc;
#else
	void *zoran_proc;
#endif
	int testing;
	int jpeg_error;
	int intr_counter_GIRQ1;
	int intr_counter_GIRQ0;
	int intr_counter_CodRepIRQ;
	int intr_counter_JPEGRepIRQ;
	int field_counter;
	int IRQ1_in;
	int IRQ1_out;
	int JPEG_in;
	int JPEG_out;
	int JPEG_0;
	int JPEG_1;
	int END_event_missed;
	int JPEG_missed;
	int JPEG_error;
	int num_errors;
	int JPEG_max_missed;
	int JPEG_min_missed;

	u32 last_isr;
	unsigned long frame_num;

	wait_queue_head_t test_q;

	enum zoran_map_mode map_mode;
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_stats;
	struct list_head queued_bufs;
	spinlock_t queued_bufs_lock; /* Protects queued_bufs */
	struct zr_vout_buffer *inuse[BUZ_NUM_STAT_COM];
};

static inline struct zoran *to_zoran(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct zoran, v4l2_dev);
}

static void btprint(u32 dat, u32 adr)
{
	switch(adr) {
	case ZR36057_ICR:
		pr_info("btwrite: ZR36057_ICR %x", dat);
		break;
	case ZR36057_ISR:
		pr_info("btwrite: ZR36057_ISR %x", dat);
		break;
	case ZR36057_VSSFGR:
		pr_info("btwrite: ZR36057_VSSFGR %x", dat);
		break;
	case ZR36057_VFEHCR:
		pr_info("btwrite: ZR36057_VFEHCR %x", dat);
		break;
	case ZR36057_VFEVCR:
		pr_info("btwrite: ZR36057_VFEVCR %x", dat);
		break;
	case ZR36057_VFESPFR:
		pr_info("btwrite: ZR36057_VFESPFR %x", dat);
		break;
	case ZR36057_VDCR:
		pr_info("btwrite: ZR36057_VDCR %x", dat);
		if (!(dat & ZR36057_VDCR_Triton))
			pr_info("TRITON\n");
		break;
	case ZR36057_VDBR:
		pr_info("btwrite: ZR36057_VDBR %x", dat);
		break;
	case ZR36057_VDTR:
		pr_info("btwrite: ZR36057_VDTR %x", dat);
		break;
	case ZR36057_POR:
		break;
	case ZR36057_JMC:
		pr_info("btwrite: ZR36057_JMCZ %x", dat);
		break;
	case ZR36057_MCTCR:
		pr_info("btwrite: ZR36057_MCTCR %x\n", dat);
		break;
	case ZR36057_JPC:
		pr_info("btwrite: ZR36057_JPC %x\n", dat);
		break;
	case ZR36057_VSP:
		pr_info("btwrite: ZR36057_VSP %x\n", dat);
		break;
	case ZR36057_HSP:
		pr_info("btwrite: ZR36057_HSP %x\n", dat);
		break;
	case ZR36057_FHAP:
		pr_info("btwrite: ZR36057_FHAP %x\n", dat);
		break;
	case ZR36057_FVAP:
		pr_info("btwrite: ZR36057_FVAP %x\n", dat);
		break;
	case ZR36057_GPPGCR1:
		pr_info("btwrite: ZR36057_GPPGCR1 %x", dat);
		break;
	case ZR36057_FPP:
		pr_info("btwrite: ZR36057_FPP %x", dat);
		break;
	case ZR36057_JCBA:
		pr_info("btwrite: ZR36057_JCBA %x", dat);
		break;
	case ZR36057_JCFT:
		pr_info("btwrite: ZR36057_JCFT %x", dat);
		break;
	case 0x044:
		break;
	default:
		pr_info("btwrite: %x %x", adr, dat);
	}
}

/* There was something called _ALPHA_BUZ that used the PCI address instead of
 * the kernel iomapped address for btread/btwrite.  */
#define btwrite(dat, adr)    {btprint(dat, adr); writel((dat), zr->zr36057_mem + (adr));}
#define btread(adr)         readl(zr->zr36057_mem + (adr))

#define btand(dat, adr)      btwrite((dat) & btread(adr), adr)
#define btor(dat, adr)       btwrite((dat) | btread(adr), adr)
#define btaor(dat, mask, adr) btwrite((dat) | ((mask) & btread(adr)), adr)

#endif

#ifndef ZORAN_OLD
int zoran_queue_init(struct zoran *zr, struct vb2_queue *vq);
#endif
