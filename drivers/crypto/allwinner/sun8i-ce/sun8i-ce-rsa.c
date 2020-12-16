// SPDX-License-Identifier: GPL-2.0
/*
 * sun8i-ce-rsa.c - hardware cryptographic accelerator for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T SoC
 *
 * Copyright (C) 2016-2021 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * This file add support for RSA operations
 *
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */
#include <linux/crypto.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/rsa.h>
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
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct akcipher_alg *alg = crypto_akcipher_alg(tfm);
	struct sun8i_ce_alg_template *algt;
	int err;

	algt = container_of(alg, struct sun8i_ce_alg_template, alg.rsa);
	ctx->ce = algt->ce;

	ctx->fallback_tfm = crypto_alloc_akcipher("rsa", 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback_tfm)) {
		dev_err(ctx->ce->dev, "ERROR: Cannot allocate fallback\n");
		return PTR_ERR(ctx->fallback_tfm);
	}

	akcipher_set_reqsize(tfm, sizeof(struct sun8i_rsa_req_ctx));

	ctx->enginectx.op.do_one_request = handle_rsa_request;
	ctx->enginectx.op.prepare_request = NULL;
	ctx->enginectx.op.unprepare_request = NULL;

	err = pm_runtime_get_sync(algt->ce->dev);
	if (err < 0)
		goto error_pm;

	return 0;
error_pm:
	pm_runtime_put_noidle(algt->ce->dev);
	crypto_free_akcipher(ctx->fallback_tfm);
	return err;
}

void sun8i_rsa_exit(struct crypto_akcipher *tfm)
{
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);

	crypto_free_akcipher(ctx->fallback_tfm);

	kfree(ctx->rsa_priv_key);
	pm_runtime_put_sync_suspend(ctx->ce->dev);
}

static void caam_rsa_drop_leading_zeros(const u8 **ptr, size_t *nbytes)
{
	while (!**ptr && *nbytes) {
		(*ptr)++;
		(*nbytes)--;
	}
}

static inline u8 *caam_read_raw_data(const u8 *buf, size_t *nbytes)
{
	caam_rsa_drop_leading_zeros(&buf, nbytes);
	if (!*nbytes)
		return NULL;

	return kmemdup(buf, *nbytes, GFP_DMA | GFP_KERNEL);
}

static void invert(u8 *src, size_t len, u8 *dst)
{
	size_t i;

	for (i = 0; i < len; i++)
		dst[i] = src[len - i - 1];
}

static int sun8i_ce_rsa_fallback(struct akcipher_request *req, int dir)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct akcipher_request *freq;
	int err;

	if (ctx->rsa_priv_key) {
		err = crypto_akcipher_set_priv_key(ctx->fallback_tfm,
						   ctx->rsa_priv_key,
						   ctx->key_len);
	} else if (ctx->rsa_pub_key) {
		err = crypto_akcipher_set_pub_key(ctx->fallback_tfm,
						  ctx->rsa_pub_key,
						  ctx->key_len);
	} else {
		return -EINVAL;
	}
	if (err)
		return err;

	freq = akcipher_request_alloc(ctx->fallback_tfm, GFP_KERNEL);
	if (!freq)
		return -ENOMEM;
	akcipher_request_set_crypt(freq, req->src, req->dst,
			req->src_len, req->dst_len);
	if (dir == CE_DECRYPTION)
		err = crypto_akcipher_decrypt(freq);
	else
		err = crypto_akcipher_encrypt(freq);
	if (err)
		return err;
	akcipher_request_free(freq);
	return 0;
}

/* IV is pubmodulus
 *
 * mode MUL(2) IV size
 * mode EXP(0) key size
 */
static int sun8i_rsa_operation(struct akcipher_request *req, int dir)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	int flow = 0;
	struct ce_task *cet;
	struct sun8i_ce_dev *ce = ctx->ce;
	int err = 0;
	void *lsrc, *key, *tmp, *ldst, *modulus;
	size_t blk_size;
	struct akcipher_alg *alg = crypto_akcipher_alg(tfm);
	struct sun8i_ce_alg_template *algt;
	bool need_fallback = false;
	dma_addr_t a_key, a_mod, a_src, a_dst;
	u32 v, common;

	algt = container_of(alg, struct sun8i_ce_alg_template, alg.rsa);

	dev_info(ctx->ce->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u dir=%d sg=%d/%d\n",
		__func__,
		ctx->raw_key.n_sz, ctx->raw_key.e_sz, ctx->raw_key.d_sz,
		ctx->raw_key.n_sz,
		req->src_len, req->dst_len, dir,
		sg_nents(req->src), sg_nents(req->dst));

	cet = ctx->ce->chanlist[flow].tl;
	memset(cet, 0, sizeof(struct ce_task));

	cet->t_id = cpu_to_le32(flow);
	common = ce->variant->alg_akcipher[algt->ce_algo_id] | CE_COMM_INT;

	blk_size = ctx->raw_key.n_sz;
	dev_info(ce->dev, "Modulus size %zu (RSA %zu)\n", blk_size, blk_size * 8);
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_req++;
#endif

	switch (blk_size * 8) {
	case 512:
		v = ce->variant->rsa_op_mode[CE_ID_RSA_512];
		break;
	case 1024:
		v = ce->variant->rsa_op_mode[CE_ID_RSA_1024];
		break;
	case 2048:
		v = ce->variant->rsa_op_mode[CE_ID_RSA_2048];
		break;
	case 3072:
		v = ce->variant->rsa_op_mode[CE_ID_RSA_3072];
		break;
	case 4096:
		v = ce->variant->rsa_op_mode[CE_ID_RSA_4096];
		break;
	default:
		v = CE_ID_NOTSUPP;
	}
	if (v == CE_ID_NOTSUPP) {
		need_fallback = true;
		dev_info(ce->dev, "Fallback due to unsupported keysize %zd\n",
			 blk_size * 8);
	}
	cet->t_asym_ctl = cpu_to_le32(v);

	/* check if fallback is necessary */
	if (need_fallback) {
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
		algt->stat_fb++;
#endif
		dev_info(ce->dev, "Fallback RSA\n");
		return sun8i_ce_rsa_fallback(req, dir);
	}

	modulus = kzalloc(ctx->raw_key.n_sz, GFP_KERNEL | GFP_DMA);
	if (!modulus)
		return -ENOMEM;

	tmp = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
	if (!tmp) {
		err = -ENOMEM;
		goto err_tmp;
	}

	lsrc = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
	if (!lsrc) {
		err = -ENOMEM;
		goto err_lsrc;
	}

	ldst = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
	if (!ldst) {
		err = -ENOMEM;
		goto err_ldst;
	}

	memcpy(tmp, ctx->modulus, ctx->raw_key.n_sz);
	/* invert modulus */
	invert(tmp, blk_size, modulus);

	/* key is exponant(encrypt) or d(decrypt) */
	if (dir == CE_ENCRYPTION) {
		key = kzalloc(blk_size, GFP_KERNEL | GFP_DMA);
		if (!key) {
			err = -ENOMEM;
			goto err_key;
		}
		caam_rsa_drop_leading_zeros(&ctx->raw_key.e, &ctx->raw_key.e_sz);
		memcpy(key, ctx->raw_key.e, ctx->raw_key.e_sz);
	} else {
		key = caam_read_raw_data(ctx->raw_key.d, &ctx->raw_key.d_sz);
		if (!key) {
			err = -ENOMEM;
			goto err_key;
		}
		memcpy(tmp, key, blk_size);
		invert(tmp, blk_size, key);
		common |= CE_DECRYPTION;
	}
	cet->t_common_ctl = cpu_to_le32(common);

	/* exposant set as key */
	a_key = dma_map_single(ce->dev, key, blk_size, DMA_TO_DEVICE);
	if (dma_mapping_error(ce->dev, a_key)) {
		dev_err(ce->dev, "Cannot DMA MAP KEY\n");
		err = -EFAULT;
		goto err_dma_key;
	}
	cet->t_key = cpu_to_le32(a_key);

	/* modulus set as IV */
	a_mod = dma_map_single(ce->dev, modulus, blk_size, DMA_TO_DEVICE);
	if (dma_mapping_error(ce->dev, a_mod)) {
		dev_err(ce->dev, "Cannot DMA MAP IV\n");
		err = -EFAULT;
		goto err_dma_mod;
	}
	cet->t_iv = cpu_to_le32(a_mod);

	err = sg_copy_to_buffer(req->src, sg_nents(req->src), tmp,
				req->src_len);
	/* invert src */
	invert(tmp, req->src_len, lsrc);

	a_src = dma_map_single(ce->dev, lsrc, blk_size, DMA_TO_DEVICE);
	if (dma_mapping_error(ce->dev, a_src)) {
		dev_err(ce->dev, "Cannot DMA MAP SRC\n");
		err = -EFAULT;
		goto err_dma_src;
	}

	a_dst = dma_map_single(ce->dev, ldst, blk_size, DMA_FROM_DEVICE);
	if (dma_mapping_error(ce->dev, a_src)) {
		dev_err(ce->dev, "Cannot DMA MAP dst\n");
		err = -EFAULT;
		goto err_dma_dst;
	}

	req->dst_len = blk_size;
	cet->t_src[0].addr = cpu_to_le32(a_src);
	cet->t_dst[0].addr = cpu_to_le32(a_dst);
	cet->t_dst[0].len = cpu_to_le32(blk_size / 4);
	cet->t_src[0].len = blk_size / 4;
	cet->t_dlen = blk_size / 4;

	if (ce->variant->rsa_in_src) {
		/* H6 store differently RSA, and use only source SG for parameters*/
		cet->t_asym_ctl = cpu_to_le32(blk_size / 4);
		/* SG0 is key, then modulus, then data */
		cet->t_src[2].addr = cet->t_src[0].addr;
		cet->t_src[0].addr = cet->t_key;
		cet->t_src[1].addr = cet->t_iv;
		cet->t_src[1].len = cpu_to_le32(blk_size / 4);
		cet->t_src[2].len = cpu_to_le32(blk_size / 4);
		cet->t_dlen = cpu_to_le32(blk_size * 3);
	}
	/*dev_info(ce->dev, "RSA: common=%x sym=%x ak=%x blk_size=%zd tdlen=%u id=%d sgsize=%d %d %d %d\n",
		cet->t_common_ctl, cet->t_sym_ctl, cet->t_asym_ctl, blk_size, cet->t_dlen, cet->t_id,
		cet->t_src[0].len, cet->t_src[1].len, cet->t_src[2].len, cet->t_dst[0].len);*/

	ctx->ce->chanlist[flow].timeout = 1000;
	err = sun8i_ce_run_task(ce, flow, "RSA");

	dma_unmap_single(ce->dev, a_dst, blk_size, DMA_FROM_DEVICE);
err_dma_dst:
	dma_unmap_single(ce->dev, a_src, blk_size, DMA_TO_DEVICE);
err_dma_src:
	dma_unmap_single(ce->dev, a_mod, blk_size, DMA_TO_DEVICE);
err_dma_mod:
	dma_unmap_single(ce->dev, a_key, blk_size, DMA_TO_DEVICE);

	if (!err) {
		/* invert DST */
		invert(ldst, blk_size, tmp);
		sg_copy_from_buffer(req->dst, sg_nents(req->dst), tmp,
				    req->dst_len);
	}

err_dma_key:
	kfree(key);
err_key:
	kfree(ldst);
err_ldst:
	kfree(lsrc);
err_lsrc:
	kfree(tmp);
err_tmp:
	kfree(modulus);
	return err;
}

int sun8i_rsa_encrypt(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = sun8i_ce_get_engine_number(ctx->ce);
	struct crypto_engine *engine = ctx->ce->chanlist[e].engine;

	dev_dbg(ctx->ce->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u\n",
		__func__,
		ctx->raw_key.n_sz, ctx->raw_key.e_sz, ctx->raw_key.d_sz,
		ctx->raw_key.n_sz,
		req->src_len, req->dst_len);
	rsa_req_ctx->op_dir = CE_ENCRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);
}

int sun8i_rsa_decrypt(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = sun8i_ce_get_engine_number(ctx->ce);
	struct crypto_engine *engine = ctx->ce->chanlist[e].engine;

	dev_dbg(ctx->ce->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u\n",
		__func__,
		ctx->raw_key.n_sz, ctx->raw_key.e_sz, ctx->raw_key.d_sz,
		ctx->raw_key.n_sz,
		req->src_len, req->dst_len);
	rsa_req_ctx->op_dir = CE_DECRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);
}

int sun8i_rsa_sign(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = sun8i_ce_get_engine_number(ctx->ce);
	struct crypto_engine *engine = ctx->ce->chanlist[e].engine;

	dev_dbg(ctx->ce->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u\n",
		__func__,
		ctx->raw_key.n_sz, ctx->raw_key.e_sz, ctx->raw_key.d_sz,
		ctx->raw_key.n_sz,
		req->src_len, req->dst_len);
	rsa_req_ctx->op_dir = CE_DECRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);
}

int sun8i_rsa_verify(struct akcipher_request *req)
{
	struct crypto_akcipher *tfm = crypto_akcipher_reqtfm(req);
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct sun8i_rsa_req_ctx *rsa_req_ctx = akcipher_request_ctx(req);
	int e = sun8i_ce_get_engine_number(ctx->ce);
	struct crypto_engine *engine = ctx->ce->chanlist[e].engine;

	dev_dbg(ctx->ce->dev, "%s modulus %zu e=%zu d=%zu c=%zu slen=%u dlen=%u\n",
		__func__,
		ctx->raw_key.n_sz, ctx->raw_key.e_sz, ctx->raw_key.d_sz,
		ctx->raw_key.n_sz,
		req->src_len, req->dst_len);
	rsa_req_ctx->op_dir = CE_ENCRYPTION;
	return crypto_transfer_akcipher_request_to_engine(engine, req);
}

int sun8i_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
			   unsigned int keylen)
{
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	int ret;

	dev_dbg(ctx->ce->dev, "%s %u\n", __func__, keylen);
	/* copy key for fallback */
	kfree(ctx->rsa_pub_key);
	ctx->rsa_pub_key = NULL;
	ctx->rsa_priv_key = kzalloc(keylen, GFP_KERNEL);
	if (!ctx->rsa_priv_key)
		return -ENOMEM;
	memcpy(ctx->rsa_priv_key, key, keylen);
	ctx->key_len = keylen;
	/* end fallback stuff */

	memset(&ctx->raw_key, 0, sizeof(struct rsa_key));
	ret = rsa_parse_priv_key(&ctx->raw_key, key, keylen);
	if (ret) {
		dev_err(ctx->ce->dev, "Invalid private key\n");
		return ret;
	}

	ctx->modulus = caam_read_raw_data(ctx->raw_key.n, &ctx->raw_key.n_sz);
	if (!ctx->modulus)
		return -ENOMEM;

	return 0;
}

int sun8i_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
			  unsigned int keylen)
{
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);
	int ret;

	dev_dbg(ctx->ce->dev, "%s %u\n", __func__, keylen);
	/* copy key for fallback */
	kfree(ctx->rsa_priv_key);
	ctx->rsa_priv_key = NULL;
	ctx->rsa_pub_key = kzalloc(keylen, GFP_KERNEL);
	if (!ctx->rsa_pub_key)
		return -ENOMEM;
	memcpy(ctx->rsa_pub_key, key, keylen);
	ctx->key_len = keylen;
	/* end fallback stuff */

	memset(&ctx->raw_key, 0, sizeof(struct rsa_key));
	ret = rsa_parse_pub_key(&ctx->raw_key, key, keylen);
	if (ret) {
		dev_err(ctx->ce->dev, "Invalid public key\n");
		return ret;
	}

	ctx->modulus = caam_read_raw_data(ctx->raw_key.n, &ctx->raw_key.n_sz);
	if (!ctx->modulus)
		return -ENOMEM;

	return 0;
}

unsigned int sun8i_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct sun8i_rsa_tfm_ctx *ctx = akcipher_tfm_ctx(tfm);

	return ctx->raw_key.n_sz;
}
