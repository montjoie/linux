/* SPDX-License-Identifier: GPL-2.0 */
/*
 * amlogic-crypto.h - hardware cryptographic accelerator for
 * Amlogic SoC
 *
 * Copyright (C) 2018 Corentin LABBE <clabbe@baylibre.com>
 */
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/engine.h>
#include <crypto/skcipher.h>
#include <linux/debugfs.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>

#define MODE_KEY 1
#define MODE_AES_128 0x8
#define MODE_AES_192 0x9
#define MODE_AES_256 0xa

#define MESON_DECRYPT 0
#define MESON_ENCRYPT 1

#define MESON_OPMODE_ECB 0
#define MESON_OPMODE_CBC 1

/*
 * struct meson_dsec - Descriptor for DMA operations
 * Note that without datasheet, some are unknown
 * @len:	length of data to operate
 * @irq:	Ignored by hardware
 * @eoc:	End of descriptor
 * @loop:	Unknown
 * @mode:	Type of algorithm (AES, SHA)
 * @begin:	Unknown
 * @end:	Unknown
 * @op_mode:	Blockmode (CBC, ECB)
 * @block:	Unknown
 * @error:	Unknown
 * @owner:	owner of the descriptor, 1 own by HW
 * @t_src:	Physical address of data to read
 * @t_dst:	Physical address of data to write
 */
struct meson_desc {
	union {
		u32 t_status;
		struct {
			u32 len:17;
			u32 irq:1;
			u32 eoc:1;
			u32 loop:1;
			u32 mode:4;
			u32 begin:1;
			u32 end:1;
			u32 op_mode:2;
			u32 enc:1;
			u32 block:1;
			u32 error:1;
			u32 owner:1;
		};
	};
	u32 t_src;
	u32 t_dst;
};

/*
 * struct meson_flow - Information used by each flow
 * @engine:	ptr to the crypto_engine for this flow
 * @keylen:	keylen for this flow operation
 * @bkeyiv:	buffer which contain the KEY and IV
 * @complete:	completion for the current task on this flow
 * @status:	set to 1 by interrupt if task is done
 * @t_phy:	Physical address of task
 * @tl:		pointer to the current ce_task for this flow
 * @stat_req:	number of request done by this flow
 */
struct meson_flow {
	struct crypto_engine *engine;
	struct completion complete;
	int status;
	unsigned int keylen;
	void *bkeyiv;
	dma_addr_t t_phy;
	struct meson_desc *tl;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	unsigned long stat_req;
#endif
};

/*
 * struct meson_variant - Describe CE capability for each variant hardware
 * @maxflow:	Numbers of flow for the current engine
 */
struct meson_variant {
	unsigned int maxflow;
};

/*
 * struct mesone_dev - main container for all this driver information
 * @base:	base address of amlogic-crypto
 * @busclk:	bus clock for amlogic-crypto
 * @reset:	pointer to reset controller
 * @dev:	the platform device
 * @chanlist:	array of all flow
 * @flow:	flow to use in next request
 * @irqs:	IRQ numbers for amlogic-crypto
 * @variant:	pointer to variant specific data
 * @dbgfs_dir:	Debugfs dentry for statistic directory
 * @dbgfs_stats: Debugfs dentry for statistic counters
 */
struct meson_dev {
	void __iomem *base;
	struct clk *busclk;
	struct reset_control *reset;
	struct device *dev;
	struct meson_flow *chanlist;
	atomic_t flow;
	int *irqs;
	const struct meson_variant *variant;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_stats;
#endif
};

/*
 * struct sun8i_cipher_req_ctx - context for a skcipher request
 * @op_dir:	direction (encrypt vs decrypt) for this request
 * @flow:	the flow to use for this request
 */
struct meson_cipher_req_ctx {
	u32 op_dir;
	int flow;
};

/*
 * struct meson_cipher_tfm_ctx - context for a skcipher TFM
 * @enginectx:		crypto_engine used by this TFM
 * @key:		pointer to key data
 * @keylen:		len of the key
 * @keymode:		The keymode(type and size of key) associated with this TFM
 * @mc:			pointer to the private data of driver handling this TFM
 * @fallback_tfm:	pointer to the fallback TFM
 */
struct meson_cipher_tfm_ctx {
	struct crypto_engine_ctx enginectx;
	u32 *key;
	u32 keylen;
	u32 keymode;
	struct meson_dev *mc;
	struct crypto_sync_skcipher *fallback_tfm;
};

/*
 * struct meson_alg_template - crypto_alg template
 * @type:		the CRYPTO_ALG_TYPE for this template
 * @blockmode:		the type of block operation
 * @mc:			pointer to the meson_dev structure associated with this template
 * @alg:		one of sub struct must be used
 * @stat_req:		number of request done on this template
 * @stat_fb:		total of all data len done on this template
 */
struct meson_alg_template {
	u32 type;
	u32 blockmode;
	union {
		struct skcipher_alg skcipher;
	} alg;
	struct meson_dev *mc;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	unsigned long stat_req;
	unsigned long stat_fb;
#endif
};

int meson_enqueue(struct crypto_async_request *areq, u32 type);

int meson_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
		     unsigned int keylen);
int meson_cipher_init(struct crypto_tfm *tfm);
void meson_cipher_exit(struct crypto_tfm *tfm);
int meson_skdecrypt(struct skcipher_request *areq);
int meson_skencrypt(struct skcipher_request *areq);
