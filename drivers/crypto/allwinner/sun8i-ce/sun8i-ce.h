#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/engine.h>
#include <crypto/skcipher.h>
#include <linux/debugfs.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>

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

/* Operation direction */
#define SS_ENCRYPTION		0
#define SS_DECRYPTION		BIT(6)
#define CE_ENCRYPTION		0
#define CE_DECRYPTION		BIT(8)

/* CE Method H3/A64 */
#define CE_ALG_AES		0
#define CE_ALG_DES		1
#define CE_ALG_3DES		2
#define CE_ALG_MD5		16
#define CE_ALG_SHA1		17
#define CE_ALG_SHA224		18
#define CE_ALG_SHA256		19
#define CE_ALG_SHA384		20
#define CE_ALG_SHA512		21
#define CE_ALG_RSA		32
#define CE_ALG_TRNG		48
#define CE_ALG_PRNG		49

#define CE_COMM_INT		BIT(31)

/* SS Method A83T */
#define SS_ALG_AES		0
#define SS_ALG_DES		(1 << 2)
#define SS_ALG_3DES		(2 << 2)
#define SS_ALG_MD5		(3 << 2)
#define SS_ALG_PRNG		(4 << 2)
#define SS_ALG_TRNG		(5 << 2)
#define SS_ALG_SHA1		(6 << 2)
#define SS_ALG_SHA224		(7 << 2)
#define SS_ALG_SHA256		(8 << 2)

/* A80/A83T SS Registers */
#define SS_CTL_REG		0x00
#define SS_INT_CTL_REG		0x04
#define SS_INT_STA_REG		0x08
#define SS_KEY_ADR_REG		0x10
#define SS_IV_ADR_REG		0x18
#define SS_SRC_ADR_REG		0x20
#define SS_DST_ADR_REG		0x28
#define SS_LEN_ADR_REG		0x30
#define SS_CTR_REG0	0x34
#define SS_CTR_REG1	0x48

#define CE_ID_NOTSUPP		0xFF

#define CE_ID_CIPHER_AES	1
#define CE_ID_CIPHER_DES	2
#define CE_ID_CIPHER_DES3	3
#define CE_ID_CIPHER_MAX	4

#define CE_ID_OP_ECB	1
#define CE_ID_OP_CBC	2
#define CE_ID_OP_CTR	3
#define CE_ID_OP_CTS	4
#define CE_ID_OP_OFB	5
#define CE_ID_OP_CFB	6
#define CE_ID_OP_CBCMAC	7
#define CE_ID_OP_MAX	8

#define CE_AES_128BITS 0
#define CE_AES_192BITS 1
#define CE_AES_256BITS 2

#define CE_OP_ECB	0
#define CE_OP_CBC	(1 << 8)
#define CE_OP_CTR	(2 << 8)
#define CE_OP_CTS	(3 << 8)

#define SS_OP_ECB	0
#define SS_OP_CBC	(1 << 13)
#define SS_OP_CTR	(2 << 14)
#define SS_OP_CTS	(3 << 14)

#define CE_CTR_128	(3 << 2)
#define SS_CTR_128	(3 << 11)
#define CE_CTS		BIT(16)

#define SS_FLOW0	BIT(30)
#define SS_FLOW1	BIT(31)

#define SS_RNG_CONTINUE	BIT(18)

#define TRNG_DATA_SIZE (256 / 8)
#define PRNG_DATA_SIZE (160 / 8)
#define PRNG_SEED_SIZE DIV_ROUND_UP(175,8)

#define CE_ARBIT_IV	BIT(16)
#define SS_ARBIT_IV	BIT(17)

#define MAXCHAN 4
#define MAX_SG 8

/* struct ce_variant - Describe CE capability for each variant hardware
 * @alg_cipher:	list of supported ciphers
 * @op_mode:	list of supported block modes
 * @is_ss:	True if the hardware is SecuritySystem
 * @intreg:	reg offset for Interrupt register
 * @maxflow:	Numbers of flow for the current engine
 */
struct ce_variant {
	char alg_cipher[CE_ID_CIPHER_MAX];
	u32 op_mode[CE_ID_OP_MAX];
	bool is_ss;
	u32 intreg;
	unsigned int maxflow;
};

struct sginfo {
	u32 addr;
	u32 len;
} __packed;

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

/* struct sun8i_ce_flow - Information used by each flow
 * @status:	set to 1 by interrupt
 * @t_phy:	Physical address of task
 */
struct sun8i_ce_flow {
	/* TODO comment on lock */
	struct mutex lock;
	struct crypto_engine *engine;
	/* IV to use */
	void *bounce_iv;
	void *next_iv;
	unsigned int ivlen;
	struct completion complete;
	int status;
	u32 method;
	u32 op_dir;
	u32 op_mode;
	unsigned int keylen;
	/* number of SG to handle in this channel */
	int nbsg;
	dma_addr_t t_phy;
	struct ce_task *tl;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	unsigned long stat_req;
#endif
};

struct sun8i_ss_ctx {
	void __iomem *base;
	void __iomem *nsbase;
	int ns_irq;
	struct clk *busclk;
	struct clk *ssclk;
	struct reset_control *reset;
	struct device *dev;
	struct resource *res;
	struct mutex mlock; /* control the use of the device */
	struct sun8i_ce_flow *chanlist;
	int flow; /* flow to use in next request */
	const struct ce_variant *variant;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	struct dentry *dbgfs_dir;
	struct dentry *dbgfs_stats;
#endif
};

struct sun8i_cipher_req_ctx {
	u32 op_dir;
	int flow;
};

struct sun8i_tfm_ctx {
	struct crypto_engine_reqctx enginectx;
	u32 *key;
	u32 keylen;
	u32 keymode;
	struct sun8i_ss_ctx *ss;
	struct crypto_skcipher *fallback_tfm;
};

struct sun8i_ss_alg_template {
	u32 type;
	u32 mode;
	u32 ce_algo_id;
	u32 ce_blockmode;
	const void *hash_init;
	union {
		struct skcipher_alg skcipher;
	} alg;
	struct sun8i_ss_ctx *ss;
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	unsigned long stat_req;
	unsigned long stat_fb;
#endif
};

int sun8i_ce_cipher(struct skcipher_request *areq);
int sun8i_ce_enqueue(struct crypto_async_request *areq, u32 type);

int sun8i_ce_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen);
int sun8i_ce_des3_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen);
int sun8i_ce_cipher_init(struct crypto_tfm *tfm);
void sun8i_ce_cipher_exit(struct crypto_tfm *tfm);
int sun8i_ce_skdecrypt(struct skcipher_request *areq);
int sun8i_ce_skencrypt(struct skcipher_request *areq);
int handle_cipher_request(struct crypto_engine *engine,
			  struct crypto_async_request *breq);

int get_engine_number(struct sun8i_ss_ctx *ss);

int sun8i_ce_run_task(struct sun8i_ss_ctx *ss, int flow, const char *name);

