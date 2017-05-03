/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sun8i-ce-cipher.c - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T SoC
 *
 * Copyright (C) 2016-2018 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * This file add support for RSA operations
 *
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */
#include <linux/crypto.h>
#include <linux/module.h>
#include <crypto/scatterwalk.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/rsa.h>
#include <linux/dma-mapping.h>
#include "sun8i-ce.h"

/* The data should be presented in a form of array of the key size
 * (modulus, key, data) such as : [LSB....MSB]
 * and the result will be return following the same pattern
 * the key (exposant) buffer is not reversed [MSB...LSB]
 * (in contrary to other data such as modulus and encryption buffer
 */
static int sun8i_rsa_operation(struct akcipher_request *req, int dir);

static int handle_rsa_request(struct crypto_engine *engine,
			      void *areq)
{
	int err;
	struct akcipher_request *req = container_of(areq, struct akcipher_request, base);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int opdir;

	opdir = rsa_req_ctx->op_dir;

	err = sun8i_rsa_operation(req, opdir);
	crypto_finalize_akcipher_request(engine, req, err);
	return 0;
}

int sun8i_rsa_init(struct crypto_akcipher *tfm)
{
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct akcipher_alg *alg = crypto_akcipher_alg(tfm);
	struct sun8i_ss_alg_template *algt;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rsa);
	ctx->ss = algt->ss;

	dev_info(ctx->ss->dev, "%s\n", __func__);

	ctx->fallback = crypto_alloc_akcipher("rsa", 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback)) {
		dev_err(ctx->ss->dev, "ERROR: Cannot allocate fallback\n");
		return PTR_ERR(ctx->fallback);
	}
	/*dev_info(ctx->ss->dev, "Use %s as fallback\n", ctx->fallback->base.cra_driver_name);*/

	akcipher_set_reqsize(tfm, sizeof(struct sun8i_rsa_req_ctx));

	ctx->enginectx.op.do_one_request = handle_rsa_request;
	ctx->enginectx.op.prepare_request = NULL;
	ctx->enginectx.op.unprepare_request = NULL;
	return 0;
}

void sun8i_rsa_exit(struct crypto_akcipher *tfm)
{
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	dev_info(ctx->ss->dev, "%s\n", __func__);
	crypto_free_akcipher(ctx->fallback);

	kfree(ctx->rsa_priv_key);
}

static inline u8 *caam_read_raw_data(const u8 *buf, size_t *nbytes)
{
	u8 *val;

	while (!*buf && *nbytes) {
		buf++;
		(*nbytes)--;
	}

	val = kzalloc(*nbytes, GFP_DMA | GFP_KERNEL);
	if (!val)
		return NULL;

	memcpy(val, buf, *nbytes);
	return val;
}

static void padd(u8 *src, size_t len, u8 *tmp)
{
	int i;

	/*pr_info("padd %zd\n", len);*/
	memcpy(tmp, src, len);
	for (i = 0; i < len; i++)
		src[i] = tmp[len - i - 1];
}

/* IV is pubmodulus
 *
 * mode MUL(2) IV size
 * mode EXP(0) key size
 * TODO check align
 */
static int sun8i_rsa_operation(struct akcipher_request *req, int dir)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	int flow = 0;
	struct ce_task *cet;
	struct sun8i_ss_ctx *ss = ctx->ss;
	int err = 0;
	u8 *modulus = NULL;
	int nr_sgd;
	int i;
	unsigned int todo, len;
	struct scatterlist *sg;
	void *sgb = NULL, *key = NULL, *tmp = NULL;
	u8 *s, *t;
	struct akcipher_request *freq;
	size_t blk_size;
	struct akcipher_alg *alg = crypto_akcipher_alg(tfm);
	struct sun8i_ss_alg_template *algt;
	bool need_fallback = false;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.rsa);

	dev_info(ctx->ss->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u\n",
		 __func__,
		 ctx->rsa_key.n_sz, ctx->rsa_key.e_sz, ctx->rsa_key.d_sz,
		 ctx->rsa_key.n_sz,
		 req->src_len, req->dst_len);

	cet = ctx->ss->chanlist[flow].tl;
	memset(cet, 0, sizeof(struct ce_task));

	cet->t_id = flow;
	cet->t_common_ctl = ss->variant->alg_akcipher[algt->ce_algo_id] | CE_COMM_INT;
	ctx->ss->chanlist[flow].method = ss->variant->alg_akcipher[algt->ce_algo_id];
#define RSA_LENDIV 4

	blk_size = ctx->rsa_key.n_sz;
	modulus = caam_read_raw_data(ctx->rsa_key.n, &blk_size);
	if (!modulus) {
		dev_err(ss->dev, "Cannot get modulus\n");
		err = -EFAULT;
		goto theend;
	}

	dev_dbg(ss->dev, "Final modulus size %zu (RSA %zu)\n", blk_size,
		blk_size * 8);
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_req++;
#endif

	switch (blk_size * 8) {
	case 512:
		cet->t_asym_ctl = ss->variant->rsa_op_mode[CE_ID_RSA_512];
		dev_info(ss->dev, "RSA 512\n");
		break;
	case 1024:
		cet->t_asym_ctl = ss->variant->rsa_op_mode[CE_ID_RSA_1024];
		dev_info(ss->dev, "RSA 1024\n");
		break;
	case 2048:
		cet->t_asym_ctl = ss->variant->rsa_op_mode[CE_ID_RSA_2048];
		dev_info(ss->dev, "RSA 2048\n");
		break;
	case 3072:
		cet->t_asym_ctl = ss->variant->rsa_op_mode[CE_ID_RSA_3072];
		dev_info(ss->dev, "RSA 3072\n");
		break;
	case 4096:
		cet->t_asym_ctl = ss->variant->rsa_op_mode[CE_ID_RSA_4096];
		dev_info(ss->dev, "RSA 4096\n");
		break;
	default:
		dev_info(ss->dev, "RSA invalid keysize\n");
		/* TODO */
	}
	ctx->ss->chanlist[flow].op_mode = cet->t_asym_ctl;
	if (cet->t_asym_ctl == CE_ID_NOTSUPP) {
		dev_info(ss->dev, "Unsupported size\n");
		need_fallback = true;
	}

	/* check if fallback is necessary */
	if (req->src_len != blk_size ||
	    blk_size > ss->variant->maxrsakeysize / 8 ||
	    need_fallback ||
	    (dir == CE_DECRYPTION && blk_size * 8 == 1024)) {
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
		algt->stat_fb++;
#endif
		dev_info(ss->dev, "Fallback %d %zd (keylen=%d)\n",
			 req->src_len, blk_size, ctx->key_len);
		if (ctx->rsa_priv_key) {
			err = crypto_akcipher_set_priv_key(ctx->fallback,
							   ctx->rsa_priv_key,
							   ctx->key_len);
		} else if (ctx->rsa_pub_key) {
			err = crypto_akcipher_set_pub_key(ctx->fallback,
							  ctx->rsa_pub_key,
							  ctx->key_len);
		} else {
			dev_err(ss->dev, "ERROR: no private or public key given\n");
			err = -EINVAL;
		}
		if (err)
			return err;

		freq = akcipher_request_alloc(ctx->fallback, GFP_KERNEL);
		if (!freq)
			return -ENOMEM;
		req->dst_len = blk_size;
		akcipher_request_set_crypt(freq, req->src, req->dst,
					   req->src_len, req->dst_len);
		if (dir == CE_DECRYPTION)
			err = crypto_akcipher_decrypt(freq);
		else
			err = crypto_akcipher_encrypt(freq);
		if (err)
			return err;
		/*dev_info(ss->dev, "Fallback end %d\n", err);*/
		/* hack fix max_size func*/
		req->dst_len = blk_size;
		akcipher_request_free(freq);
		return 0;
	}

	tmp = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
	if (!tmp)
		return -ENOMEM;
	key = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
	if (!key)
		return -ENOMEM;
	/* key is exponant(encrypt) or d(decrypt) */
	if (dir == CE_ENCRYPTION) {
		memcpy(key, ctx->rsa_key.e, ctx->rsa_key.e_sz);
#ifdef DEBUG_CE_HEX
		print_hex_dump(KERN_INFO, "EXP ", DUMP_PREFIX_NONE, 16, 1, key,
			       blk_size, false);
#endif
	} else {
		memcpy(key, ctx->rsa_key.d, ctx->rsa_key.d_sz);
		padd(key, blk_size, tmp);
		cet->t_common_ctl |= CE_DECRYPTION;
	}

	/* exposant set as key */
	cet->t_key = dma_map_single(ss->dev, key, blk_size, DMA_TO_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_key)) {
		dev_err(ss->dev, "Cannot DMA MAP KEY\n");
		err = -EFAULT;
		goto theend;
	}

	/* invert modulus */
	memcpy(tmp, modulus, blk_size);
	padd(modulus, blk_size, tmp);

	/*check_align(modulus);*/
	/* modulus set as IV */
	cet->t_iv = dma_map_single(ss->dev, modulus, blk_size, DMA_TO_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_iv)) {
		dev_err(ss->dev, "Cannot DMA MAP IV\n");
		err = -EFAULT;
		goto theend;
	}

#ifdef DEBUG_CE_HEX
	print_hex_dump(KERN_INFO, "KEY ", DUMP_PREFIX_NONE, 16, 1,
		       ctx->rsa_key.e, ctx->rsa_key.e_sz, false);
	print_hex_dump(KERN_INFO, "MOD ", DUMP_PREFIX_NONE, 16, 1, modulus,
		       blk_size, false);
#endif
	/* handle data */
	sgb = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
	if (!sgb)
		return -ENOMEM;
	err = sg_copy_to_buffer(req->src, sg_nents(req->src), sgb,
				req->src_len);
	/* invert src */
	padd(sgb, blk_size, tmp);

	/*check_align(sgb);*/
#ifdef DEBUG_CE_HEX
	print_hex_dump(KERN_INFO, "SRC ", DUMP_PREFIX_NONE, 16, 1, sgb,
		       blk_size, false);
#endif
	cet->t_src[0].addr = dma_map_single(ss->dev, sgb, blk_size,
					    DMA_TO_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_src[0].addr)) {
		dev_err(ss->dev, "Cannot DMA MAP SRC\n");
		err = -EFAULT;
		goto theend;
	}

	/* handle destination data */
	nr_sgd = dma_map_sg(ss->dev, req->dst, sg_nents(req->dst),
			    DMA_FROM_DEVICE);
	if (nr_sgd < 0) {
		dev_err(ss->dev, "Cannot DMA MAP dst\n");
		err = -EFAULT;
		goto theend;
	}

	req->dst_len = blk_size;
	len = blk_size;
	for_each_sg(req->dst, sg, nr_sgd, i) {
		cet->t_dst[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_dst[i].len = todo / RSA_LENDIV;
		dev_info(ss->dev, "DST %02d todo=%u\n", i, todo);
		len -= todo;
	}
	/*check_align(cet->t_dst[0].addr);*/

	cet->t_src[0].len = blk_size / RSA_LENDIV;
	cet->t_dlen = blk_size / RSA_LENDIV;

	dev_info(ss->dev, "SRC %u\n", cet->t_src[0].len);
	/*dev_info(ss->dev, "DST %u\n", cet->t_dst[0].len);*/
	dev_info(ss->dev, "CTL %x %x %x\n", cet->t_common_ctl, cet->t_sym_ctl,
		 cet->t_asym_ctl);

	err = sun8i_ce_run_task(ss, flow, "RSA");

	dma_unmap_single(ss->dev, cet->t_src[0].addr, blk_size, DMA_TO_DEVICE);
	dma_unmap_sg(ss->dev, req->dst, nr_sgd, DMA_FROM_DEVICE);
	dma_unmap_single(ss->dev, cet->t_key, blk_size, DMA_TO_DEVICE);
	dma_unmap_single(ss->dev, cet->t_iv, blk_size, DMA_TO_DEVICE);

	sg_copy_to_buffer(req->dst, sg_nents(req->dst), modulus, req->dst_len);

	/* invert DST */
	t = modulus;
	s = sgb;
	for (i = 0; i < blk_size; i++)
		s[i] = t[blk_size - i - 1];
	sg_copy_from_buffer(req->dst, sg_nents(req->dst), sgb, req->dst_len);

theend:
	kfree(modulus);
	kfree(tmp);
	return err;
}

int sun8i_rsa_encrypt(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = get_engine_number(ctx->ss);
	struct crypto_engine *engine = ctx->ss->chanlist[e].engine;

	dev_info(ctx->ss->dev, "%s\n", __func__);
	rsa_req_ctx->op_dir = CE_ENCRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);

	return sun8i_rsa_operation(req, CE_ENCRYPTION);
}

int sun8i_rsa_decrypt(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = get_engine_number(ctx->ss);
	struct crypto_engine *engine = ctx->ss->chanlist[e].engine;

	dev_info(ctx->ss->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u\n",
		 __func__,
		 ctx->rsa_key.n_sz, ctx->rsa_key.e_sz, ctx->rsa_key.d_sz,
		 ctx->rsa_key.n_sz,
		 req->src_len, req->dst_len);
	rsa_req_ctx->op_dir = CE_DECRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);

	return sun8i_rsa_operation(req, CE_DECRYPTION);
}

int sun8i_rsa_sign(struct akcipher_request *req)
{
	pr_info("%s un-implemented\n", __func__);
	return 0;
}

int sun8i_rsa_verify(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = get_engine_number(ctx->ss);
	struct crypto_engine *engine = ctx->ss->chanlist[e].engine;

	dev_info(ctx->ss->dev, "%s\n", __func__);
	rsa_req_ctx->op_dir = CE_ENCRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);

	sun8i_rsa_operation(req, CE_ENCRYPTION);
	return 0;
}

int sun8i_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
			   unsigned int keylen)
{
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	int ret;

	dev_info(ctx->ss->dev, "%s keylen=%u\n", __func__, keylen);

	kfree(ctx->rsa_pub_key);
	ctx->rsa_pub_key = NULL;
	ctx->rsa_priv_key = kmalloc(keylen, GFP_KERNEL);
	if (!ctx->rsa_priv_key)
		return -ENOMEM;
	memcpy(ctx->rsa_priv_key, key, keylen);
	ctx->key_len = keylen;

	ret = rsa_parse_priv_key(&ctx->rsa_key, key, keylen);
	if (ret) {
		dev_err(ctx->ss->dev, "Invalid private key\n");
		return ret;
	}

	return 0;
}

int sun8i_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
			  unsigned int keylen)
{
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	int ret;

	dev_info(ctx->ss->dev, "%s keylen=%u\n", __func__, keylen);

	kfree(ctx->rsa_priv_key);
	ctx->rsa_priv_key = NULL;
	ctx->rsa_pub_key = kmalloc(keylen, GFP_KERNEL);
	if (!ctx->rsa_pub_key)
		return -ENOMEM;
	memcpy(ctx->rsa_pub_key, key, keylen);
	ctx->key_len = keylen;

	memset(&ctx->rsa_key, 0, sizeof(struct rsa_key));
	ret = rsa_parse_pub_key(&ctx->rsa_key, key, keylen);
	if (ret) {
		dev_err(ctx->ss->dev, "Invalid public key\n");
		return ret;
	}
	return 0;
}

unsigned int sun8i_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct sun8i_tfm_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);

	return ctx->key_len;
}
