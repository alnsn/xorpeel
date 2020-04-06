/*-
 * Copyright (c) 2015-2016 Alexander Nasonov.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#ifndef XORPEEL_GRAPH_H_INCLUDED
#define XORPEEL_GRAPH_H_INCLUDED

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Return values of xorpeel functions. */
#define XORPEEL_SUCCESS 0
#define XORPEEL_INVAL  -1 // EINVAL
#define XORPEEL_RANGE  -2 // ERANGE
#define XORPEEL_NOMEM  -3 // ENOMEM
#define XORPEEL_AGAIN  -4 // Graph has a cycle or xorpeel_find_duplicates() failed
#define XORPEEL_NOKEY  -5 // Iterator returned no key

#define	XORPEEL_RANK2     0x1
#define	XORPEEL_RANK3     0x2
#define	XORPEEL_RANK_MASK 0x3

#define XORPEEL_REDUCE_MOD    0x04 // Modulo reduction of a hash.
#define XORPEEL_REDUCE_MUL    0x08 // Alternative reduction by multiplication.
#define XORPEEL_REDUCE_CUSTOM 0x10 // Custom reduction xorpeel_reduce_t.
#define XORPEEL_REDUCE_MASK   0x1c

#define	XORPEEL_DEFAULT 0 // XORPEEL_RANK3 | XORPEEL_REDUCE_MOD

struct xorpeel_graph;

typedef uint32_t xorpeel_seed_t;

/* Very generic callback interface. */
typedef void const * (*xorpeel_iterator_t)(void *);

/* Vector hash and reduce. */
typedef void (*xorpeel_hash_t)(void const *, xorpeel_seed_t, uint32_t []);
typedef void (*xorpeel_reduce_t)(uint32_t, uint32_t []);

struct xorpeel_graph *xorpeel_alloc_graph(size_t, int);
void xorpeel_free_graph(struct xorpeel_graph *);

int xorpeel_flags(struct xorpeel_graph const *);
int xorpeel_rank(struct xorpeel_graph const *);
size_t xorpeel_num_entries(struct xorpeel_graph const *);
size_t xorpeel_num_vertices(struct xorpeel_graph const *);
size_t xorpeel_core_size(struct xorpeel_graph const *);
xorpeel_seed_t xorpeel_seed(struct xorpeel_graph const *);

int xorpeel_build_graph(struct xorpeel_graph *, xorpeel_hash_t hash,
    xorpeel_seed_t, xorpeel_iterator_t, void *, xorpeel_reduce_t);
int xorpeel_is_built(struct xorpeel_graph const *);

// XXX Copy xorpeel_find_duplicates() from rgph.
size_t xorpeel_count_keys(xorpeel_iterator_t, void *);

#ifdef __cplusplus
}
#endif

#endif /* !XORPEEL_GRAPH_H_INCLUDED */
