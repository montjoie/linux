/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sun8i-ce-cipher.c - hardware cryptographic offloader for
 * Allwinner H3/A64/H5/H2+/H6/A80/A83T SoC
 *
 * Copyright (C) 2016-2018 Corentin LABBE <clabbe.montjoie@gmail.com>
 *
 * This file add support for AES cipher with 128,192,256 bits keysize in
 * CBC and ECB mode.
 *
 * You could find a link for the datasheet in Documentation/arm/sunxi/README
 */

#include <linux/crypto.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/scatterlist.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/skcipher.h>
#include "sun8i-ce.h"

static int sun8i_ce_cipher(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ss_ctx *ss = op->ss;
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int flow;
	struct ce_task *cet;
	int nr_sgs, nr_sgd;
	struct scatterlist *sg;
	struct scatterlist *in_sg = areq->src;
	struct scatterlist *out_sg = areq->dst;
	int i;
	int err = 0;
	unsigned int todo, len;
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);
	struct sun8i_ss_alg_template *algt;
	bool need_fallback = false;

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.skcipher);

	dev_dbg(ss->dev, "%s %s %u %x IV(%p %u) key=%u\n", __func__,
		crypto_tfm_alg_name(areq->base.tfm),
		areq->cryptlen,
		rctx->op_dir, areq->iv, crypto_skcipher_ivsize(tfm),
		op->keylen);

	if (sg_nents(areq->src) > 8 || sg_nents(areq->dst) > 8)
		need_fallback = true;

	sg = areq->src;
	while (sg && !need_fallback) {
		if ((sg->length % 4) != 0)
			need_fallback = true;
		if ((sg_dma_len(sg) % 16) != 0)
			need_fallback = true;
		if (!IS_ALIGNED(sg->offset, sizeof(u32)))
			need_fallback = true;
		sg = sg_next(sg);
	}
	sg = areq->dst;
	while (sg && !need_fallback) {
		if ((sg->length % 4) != 0)
			need_fallback = true;
		if ((sg_dma_len(sg) % 4) != 0)
			need_fallback = true;
		if (!IS_ALIGNED(sg->offset, sizeof(u32)))
			need_fallback = true;
		sg = sg_next(sg);
	}

#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
	algt->stat_req++;
#endif

	/* on SS, src and dst SG must have the same len TODO */

	if (need_fallback) {
		SYNC_SKCIPHER_REQUEST_ON_STACK(subreq, op->fallback_tfm);
#ifdef CONFIG_CRYPTO_DEV_SUN8I_CE_DEBUG
		algt->stat_fb++;
#endif
		skcipher_request_set_sync_tfm(subreq, op->fallback_tfm);
		skcipher_request_set_callback(subreq, areq->base.flags, NULL,
					      NULL);
		skcipher_request_set_crypt(subreq, areq->src, areq->dst,
					   areq->cryptlen, areq->iv);
		if (rctx->op_dir & CE_DECRYPTION)
			err = crypto_skcipher_decrypt(subreq);
		else
			err = crypto_skcipher_encrypt(subreq);
		skcipher_request_zero(subreq);
		return err;
	}

	flow = rctx->flow;

	mutex_lock(&ss->chanlist[flow].lock);

	cet = ss->chanlist[flow].tl;
	memset(cet, 0, sizeof(struct ce_task));

	cet->t_id = flow;
	cet->t_common_ctl = ss->variant->alg_cipher[algt->ce_algo_id];
	cet->t_common_ctl |= rctx->op_dir | CE_COMM_INT;
	cet->t_dlen = areq->cryptlen / 4;
	/* CTS and recent CE (H6) need lenght in bytes, in word otherwise */
	if (algt->ce_blockmode == CE_ID_OP_CTS || ss->variant->model == CE_v2)
		cet->t_dlen = areq->cryptlen;

	cet->t_sym_ctl = ss->variant->op_mode[algt->ce_blockmode];
	switch (op->keylen) {
	case 128 / 8:
		cet->t_sym_ctl |= CE_AES_128BITS;
		break;
	case 192 / 8:
		cet->t_sym_ctl |= CE_AES_192BITS;
		break;
	case 256 / 8:
		cet->t_sym_ctl |= CE_AES_256BITS;
		break;
	}
	if (algt->ce_blockmode == CE_ID_OP_CTR)
		cet->t_sym_ctl |= CE_CTR_128;
	if (algt->ce_blockmode == CE_ID_OP_CTS)
		cet->t_sym_ctl |= CE_CTS;
	cet->t_asym_ctl = 0;

	ss->chanlist[flow].op_mode = ss->variant->op_mode[algt->ce_blockmode];
	if (algt->ce_blockmode == CE_ID_OP_CTR)
		ss->chanlist[flow].op_mode |= SS_CTR_128;
	ss->chanlist[flow].op_dir = rctx->op_dir;
	ss->chanlist[flow].method = ss->variant->alg_cipher[algt->ce_algo_id];
	ss->chanlist[flow].keylen = op->keylen;

	cet->t_key = dma_map_single(ss->dev, op->key, op->keylen,
				    DMA_TO_DEVICE);
	if (dma_mapping_error(ss->dev, cet->t_key)) {
		dev_err(ss->dev, "Cannot DMA MAP KEY\n");
		err = -EFAULT;
		goto theend;
	}

	if (areq->iv && crypto_skcipher_ivsize(tfm) > 0) {
		ss->chanlist[flow].ivlen = crypto_skcipher_ivsize(tfm);
		ss->chanlist[flow].bounce_iv = kzalloc(ss->chanlist[flow].ivlen,
						       GFP_KERNEL | GFP_DMA);
		if (!ss->chanlist[flow].bounce_iv) {
			err = -ENOMEM;
			goto theend;
		}
		memcpy(ss->chanlist[flow].bounce_iv, areq->iv,
		       crypto_skcipher_ivsize(tfm));
		ss->chanlist[flow].next_iv = kzalloc(ss->chanlist[flow].ivlen,
						     GFP_KERNEL | GFP_DMA);
		if (!ss->chanlist[flow].next_iv) {
			err = -ENOMEM;
			goto theend;
		}
	}

	if (in_sg == out_sg) {
		nr_sgs = dma_map_sg(ss->dev, in_sg, sg_nents(in_sg),
				    DMA_BIDIRECTIONAL);
		if (nr_sgs <= 0 || nr_sgs > 8) {
			dev_err(ss->dev, "Invalid sg number %d\n", nr_sgs);
			err = -EINVAL;
			goto theend;
		}
		nr_sgd = nr_sgs;
	} else {
		nr_sgs = dma_map_sg(ss->dev, in_sg, sg_nents(in_sg),
				    DMA_TO_DEVICE);
		if (nr_sgs <= 0 || nr_sgs > 8) {
			dev_err(ss->dev, "Invalid sg number %d\n", nr_sgs);
			err = -EINVAL;
			goto theend;
		}
		nr_sgd = dma_map_sg(ss->dev, out_sg, sg_nents(out_sg),
				    DMA_FROM_DEVICE);
		if (nr_sgd <= 0 || nr_sgd > 8) {
			dev_err(ss->dev, "Invalid sg number %d\n", nr_sgd);
			err = -EINVAL;
			goto theend;
		}
	}

	len = areq->cryptlen;
	for_each_sg(in_sg, sg, nr_sgs, i) {
		cet->t_src[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_src[i].len = todo / 4;
		dev_dbg(ss->dev, "%s total=%u SG(%d %u) todo=%u\n", __func__, areq->cryptlen, i, cet->t_src[i].len, todo);
		len -= todo;
	}

	len = areq->cryptlen;
	for_each_sg(out_sg, sg, nr_sgd, i) {
		cet->t_dst[i].addr = sg_dma_address(sg);
		todo = min(len, sg_dma_len(sg));
		cet->t_dst[i].len = todo / 4;
		dev_dbg(ss->dev, "%s total=%u SG(%d %u) todo=%u\n", __func__, areq->cryptlen, i, cet->t_dst[i].len, todo);
		len -= todo;
	}

	err = sun8i_ce_run_task(ss, flow, "cipher");

	if (areq->iv && crypto_skcipher_ivsize(tfm) > 0) {
		memcpy(areq->iv, ss->chanlist[flow].next_iv,
		       ss->chanlist[flow].ivlen);
		memzero_explicit(ss->chanlist[flow].bounce_iv,
				 ss->chanlist[flow].ivlen);
		kfree(ss->chanlist[flow].bounce_iv);
		kfree(ss->chanlist[flow].next_iv);
		ss->chanlist[flow].bounce_iv = NULL;
		ss->chanlist[flow].next_iv = NULL;
	}

	dma_unmap_single(ss->dev, cet->t_key, op->keylen, DMA_TO_DEVICE);
	if (in_sg == out_sg) {
		dma_unmap_sg(ss->dev, in_sg, nr_sgs, DMA_BIDIRECTIONAL);
	} else {
		dma_unmap_sg(ss->dev, in_sg, nr_sgs, DMA_TO_DEVICE);
		dma_unmap_sg(ss->dev, out_sg, nr_sgd, DMA_FROM_DEVICE);
	}

theend:
	mutex_unlock(&ss->chanlist[flow].lock);

	return err;
}

static int handle_cipher_request(struct crypto_engine *engine,
				 void *areq)
{
	int err;
	struct skcipher_request *breq = container_of(areq, struct skcipher_request, base);

	err = sun8i_ce_cipher(breq);
	crypto_finalize_skcipher_request(engine, breq, err);

	return 0;
}

int sun8i_ce_skdecrypt(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int e = get_engine_number(op->ss);
	struct crypto_engine *engine = op->ss->chanlist[e].engine;

	rctx->op_dir = CE_DECRYPTION;
	rctx->flow = e;

	return crypto_transfer_skcipher_request_to_engine(engine, areq);
}

int sun8i_ce_skencrypt(struct skcipher_request *areq)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(areq);
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_cipher_req_ctx *rctx = skcipher_request_ctx(areq);
	int e = get_engine_number(op->ss);
	struct crypto_engine *engine = op->ss->chanlist[e].engine;

	rctx->op_dir = CE_ENCRYPTION;
	rctx->flow = e;

	return crypto_transfer_skcipher_request_to_engine(engine, areq);
}

int sun8i_ce_cipher_init(struct crypto_tfm *tfm)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_tfm_ctx(tfm);
	struct sun8i_ss_alg_template *algt;
	const char *name = crypto_tfm_alg_name(tfm);
	struct crypto_skcipher *sktfm = __crypto_skcipher_cast(tfm);
	struct skcipher_alg *alg = crypto_skcipher_alg(sktfm);

	memset(op, 0, sizeof(struct sun8i_cipher_tfm_ctx));

	algt = container_of(alg, struct sun8i_ss_alg_template, alg.skcipher);
	op->ss = algt->ss;

	sktfm->reqsize = sizeof(struct sun8i_cipher_req_ctx);

	op->fallback_tfm = crypto_alloc_sync_skcipher(name, 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(op->fallback_tfm)) {
		dev_err(op->ss->dev, "ERROR: Cannot allocate fallback for %s %ld\n",
			name, PTR_ERR(op->fallback_tfm));
		return PTR_ERR(op->fallback_tfm);
	}

	op->enginectx.op.do_one_request = handle_cipher_request;
	op->enginectx.op.prepare_request = NULL;
	op->enginectx.op.unprepare_request = NULL;

	return 0;
}

void sun8i_ce_cipher_exit(struct crypto_tfm *tfm)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_tfm_ctx(tfm);

	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	crypto_free_sync_skcipher(op->fallback_tfm);
}

int sun8i_ce_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
			unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ss_ctx *ss = op->ss;

	switch (keylen) {
	case 128 / 8:
		break;
	case 192 / 8:
		break;
	case 256 / 8:
		break;
	default:
		dev_err(ss->dev, "ERROR: Invalid keylen %u\n", keylen);
		crypto_skcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	op->keylen = keylen;
	op->key = kmalloc(keylen, GFP_KERNEL | GFP_DMA);
	if (!op->key)
		return -ENOMEM;
	memcpy(op->key, key, keylen);

	return crypto_sync_skcipher_setkey(op->fallback_tfm, key, keylen);
}

int sun8i_ce_des3_setkey(struct crypto_skcipher *tfm, const u8 *key,
			 unsigned int keylen)
{
	struct sun8i_cipher_tfm_ctx *op = crypto_skcipher_ctx(tfm);
	struct sun8i_ss_ctx *ss = op->ss;

	if (unlikely(keylen != 3 * DES_KEY_SIZE)) {
		dev_err(ss->dev, "Invalid keylen %u\n", keylen);
		crypto_skcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	if (op->key) {
		memzero_explicit(op->key, op->keylen);
		kfree(op->key);
	}
	op->keylen = keylen;
	op->key = kmalloc(keylen, GFP_KERNEL | GFP_DMA);
	if (!op->key)
		return -ENOMEM;
	memcpy(op->key, key, keylen);

	return crypto_sync_skcipher_setkey(op->fallback_tfm, key, keylen);
}
