/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sun8i-ce.h - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6 SoC
 *
 * Copyright (C) 2016-2019 Corentin LABBE <clabbe.montjoie@gmail.com>
 */
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/engine.h>
#include <crypto/skcipher.h>
#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/crypto.h>

/* CE Registers */
#define CE_TDQ	0x00
#define CE_CTR	0x04
#define CE_ICR	0x08
#define CE_ISR	0x0C
#define CE_TLR	0x10
#define CE_TSR	0x14
#define CE_ESR	0x18
#define CE_CSSGR	0x1C
#define CE_CDSGR	0x20
#define CE_CSAR	0x24
#define CE_CDAR	0x28
#define CE_TPR	0x2C

#define CE_ENCRYPTION		0
#define CE_DECRYPTION		BIT(8)

#define CE_ALG_AES		0
#define CE_ALG_DES		1
#define CE_ALG_3DES		2
#define CE_ALG_RAES		48

#define CE_COMM_INT		BIT(31)

#define CE_ID_NOTSUPP		0xFF

#define CE_ID_CIPHER_AES	1
#define CE_ID_CIPHER_DES	2
#define CE_ID_CIPHER_DES3	3
#define CE_ID_CIPHER_RAES	4
#define CE_ID_CIPHER_MAX	5

#define CE_ID_OP_ECB	1
#define CE_ID_OP_CBC	2
#define CE_ID_OP_CTR	3
#define CE_ID_OP_CTS	4
#define CE_ID_OP_OFB	5
#define CE_ID_OP_CFB	6
#define CE_ID_OP_CBCMAC	7
#define CE_ID_OP_XTS	8
#define CE_ID_OP_MAX	9

#define CE_AES_128BITS 0
#define CE_AES_192BITS 1
#define CE_AES_256BITS 2

#define CE_OP_ECB	0
#define CE_OP_CBC	(1 << 8)
#define CE_OP_CTR	(2 << 8)
#define CE_OP_CTS	(3 << 8)
#define CE_OP_XTS	(9 << 8)

#define CE_CTR_128	(3 << 2)
#define CE_CTS		BIT(16)
#define CE_XTS_FIRST	BIT(12)
#define CE_XTS_LAST	BIT(13)

#define CE_ARBIT_IV	BIT(16)

#define CE_ERR_ALGO_NOTSUP	BIT(0)
#define CE_ERR_DATALEN		BIT(1)
#define CE_ERR_KEYSRAM		BIT(2)
#define CE_ERR_ADDR_INVALID	BIT(5)
#define CE_ERR_KEYLADDER	BIT(6)

#define MAX_SG 8

#define CE_STD 0
#define CE_v2 2

#define CE_MAX_CLOCKS 3

/*
 * struct ce_variant - Describe CE capability for each variant hardware
 * @alg_cipher:	list of supported ciphers. for each CE_ID_ this will give the
 * 	coresponding CE_ALG_XXX/SS_ALG_XXX value
 * @op_mode:	list of supported block modes
 * @model:	The minor variant CE_STD/CE_SS/CE_v2
 * @intreg:	reg offset for Interrupt register
 * @maxflow:	Numbers of flow for the current engine
 * @prng:	the ALG_ID of prng if supported
 * @maxrsakeysize:	The maximum size of RSA key supported
 * @alg_akcipher:	list of supported akciphers
 * @rsa_op_mode:	op_mode value for RSA keys
 */
struct ce_variant {
	char alg_cipher[CE_ID_CIPHER_MAX];
	u32 op_mode[CE_ID_OP_MAX];
	char op_len_align[CE_ID_OP_MAX];
	int model;
	u32 intreg;
	unsigned int maxflow;
	unsigned long clk_freqs[CE_MAX_CLOCKS];
	const char *clk_names[CE_MAX_CLOCKS];
};

struct sginfo {
	u32 addr;
	u32 len;
} __packed;

/*
 * struct ce_task - CE Task descriptor
 * The structure of this descriptor could be found in the datasheet
 */
struct ce_task {
	u32 t_id;
	u32 t_common_ctl;
	u32 t_sym_ctl;
	u32 t_asym_ctl;
	u32 t_key;
	u32 t_iv;
	u32 t_ctr;
	u32 t_dlen;
	struct sginfo t_src[MAX_SG];
	struct sginfo t_dst[MAX_SG];
	u32 next;
	u32 reserved[3];
} __packed __aligned(8);

/*
 * struct sun8i_ce_flow - Information used by each flow
 * @lock:	lock protectin access of sun8i_ce_flow
 * @engine:	ptr to the crypto_engine for this flow
 * @bounce_iv:	buffer which contain the IV
 * @next_iv:	buffer containing the next IV to use
 * @ivlen:	size of bounce_iv
 * @keylen:	keylen for this flow operation
 * @complete:	completion for the current task on this flow
 * @status:	set to 1 by interrupt if task is done
 * @method:	current method for flow
 * @op_dir:	direction (encrypt vs decrypt) of this flow
 * @op_mode:	op_mode for this flow
 * @t_phy:	Physical address of task
 * @tl:		pointer to the current ce_task for this flow
 * @stat_req:	number of request done by this flow
 */
struct sun8i_ce_flow {
	struct mutex lock;
	struct crypto_engine *engine;
	void *bounce_iv;
	void *next_iv;
	unsigned int ivlen;
	unsigned int keylen;
	struct completion complete;
	int status;
	u32 method;
	u32 op_dir;
	u32 op_mode;
	dma_addr_t t_phy;
	struct ce_task *tl;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	unsigned long stat_req;
#endif
};

/*
 * struct sun8i_ce_dev - main container for all this driver information
 * @base:	base address of SS/CE
 * @ceclks:	clocks used by SS/CE
 * @reset:	pointer to reset controller
 * @dev:	the platform device
 * @mlock:	Control access to device registers
 * @chanlist:	array of all flow
 * @flow:	flow to use in next request
 * @variant:	pointer to variant specific data
 * @dbgfs_dir:	Debugfs dentry for statistic directory
 * @dbgfs_stats: Debugfs dentry for statistic counters
 */
struct sun8i_ce_dev {
	void __iomem *base;
	struct clk *ceclks[CE_MAX_CLOCKS];
	struct reset_control *reset;
	struct device *dev;
	struct mutex mlock;
	struct sun8i_ce_flow *chanlist;
	atomic_t flow;
	const struct ce_variant *variant;
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
struct sun8i_cipher_req_ctx {
	u32 op_dir;
	int flow;
};

/*
 * struct sun8i_cipher_tfm_ctx - context for a skcipher TFM
 * @enginectx:		crypto_engine used by this TFM
 * @key:		pointer to key data
 * @keylen:		len of the key
 * @ce:			pointer to the private data of driver handling this TFM
 * @fallback_tfm:	pointer to the fallback TFM
 */
struct sun8i_cipher_tfm_ctx {
	struct crypto_engine_ctx enginectx;
	u32 *key;
	u32 keylen;
	struct sun8i_ce_dev *ce;
	struct crypto_sync_skcipher *fallback_tfm;
};

/*
 * struct sun8i_ce_alg_template - crypto_alg template
 * @type:		the CRYPTO_ALG_TYPE for this template
 * @ce_algo_id:		the CE_ID for this template
 * @ce_blockmode:	the type of block operation CE_ID
 * @ce:			pointer to the sun8i_ce_dev structure associated with
 *			this template
 * @alg:		one of sub struct must be used
 * @stat_req:		number of request done on this template
 * @stat_fb:		total of all data len done on this template
 */
struct sun8i_ce_alg_template {
	u32 type;
	u32 ce_algo_id;
	u32 ce_blockmode;
	struct sun8i_ce_dev *ce;
	union {
		struct skcipher_alg skcipher;
	} alg;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	unsigned long stat_req;
	unsigned long stat_fb;
#endif
};

int sun8i_ce_enqueue(struct crypto_async_request *areq, u32 type);

int sun8i_ce_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen);
int sun8i_ce_aes_xts_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen);
int sun8i_ce_des3_setkey(struct crypto_skcipher *tfm, const u8 *key,
			 unsigned int keylen);
int sun8i_ce_cipher_init(struct crypto_tfm *tfm);
void sun8i_ce_cipher_exit(struct crypto_tfm *tfm);
int sun8i_ce_skdecrypt(struct skcipher_request *areq);
int sun8i_ce_skencrypt(struct skcipher_request *areq);

int get_engine_number(struct sun8i_ce_dev *ce);

int sun8i_ce_run_task(struct sun8i_ce_dev *ce, int flow, const char *name);
