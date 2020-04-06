/*-
 * Copyright (c) 2015-2017, 2020 Alexander Nasonov.
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

/*
 * The algorithm below is based on paper
 * Cache-Oblivious Peeling of Random Hypergraphs by Djamal Belazzougui,
 * Paolo Boldi, Giuseppe Ottaviano, Rossano Venturini, and Sebastiano
 * Vigna.
 * https://arxiv.org/pdf/1312.0526.pdf
 */

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "fastdiv.h"
#include "xorpeel.h"
#include "bitops.h"

#define MAX_NKEYS_R2_VEC 0x78787877u
#define MAX_NKEYS_R3_VEC 0xcccccccau

namespace {

enum {
	ZEROED   = 0x40000000, // The order, edges and oedges arrays are zeroed.
	BUILT    = 0x20000000, // Graph is built.
	PEELED   = 0x10000000, // Peel order index is built.
	ASSIGNED = 0x08000000, // Assignment step is done.
	PUBLIC_FLAGS = 0xffff
};

typedef uint32_t vert_t;         // Vertex or key; V in templates.

/*
 * Generated graphs are always R-partite. This means that v0 is less
 * than (nverts / R), v1 starts from (nverts / R) and is less than
 * (2 * nverts / R). If R==3, v2 takes the remaining range.
 * All partitions in a generated graph have equal number of vertices,
 * in other words, nverts is always a multiple of R.
 */
template<class V, int R>
struct edge {
	V verts[R]; // v0, v1 (and v2, if R==3).
};

/*
 * Data type for a valid oriented edge (v0, v1, v2), v1 < v2.
 * The first vertex v0 is implicit and is determined by an index
 * of the corresponding element in the oedges array.
 * When the algorithm starts, the degree and overts are set to zeroes.
 * Every time an edge is added or removed, the edge's verts values
 * are XORed with corresponding overts values.
 */
template<class V, int R>
struct oedge {
	V overts[R-1]; // XORed v1 (and v2, if R==3).
	V degree;      // Degree of v0.
	V edge;
};

// Partition a graph using fast_remainder32(3) from NetBSD.
template<int R>
struct fastrem_reduce {
	vert_t  partsz; // Partition size of an R-partite R-graph.
	vert_t  mul;
	uint8_t s2;

	inline fastrem_reduce(size_t);

	inline void operator()(vert_t []) const;
};

// http://lemire.me/blog/2016/06/27/a-fast-alternative-to-the-modulo-reduction/
template<int R>
struct lemire_reduce {
	vert_t partsz; // Partition size of an R-partite R-graph.

	inline lemire_reduce(size_t);

	inline void operator()(vert_t []) const;
};

template<int R>
struct custom_reduce {
	vert_t partsz; // Partition size of an R-partite R-graph.
	xorpeel_reduce_t reduce;

	inline custom_reduce(size_t, xorpeel_reduce_t);

	inline void operator()(vert_t []) const;
};

} // anon namespace

struct xorpeel_graph {
	size_t nkeys;
	size_t nverts;
	void *order;  // Output order of edges, points to V[nkeys] array.
	void *edges;  // Points to edge<V,R>[nkeys] array.
	void *oedges; // Points to oedge<V,R>[nverts] array.
	size_t core_size; // R-core size.
	xorpeel_seed_t seed;
	unsigned int flags;
};

namespace {

template<class V>
inline void
add_remove_oedge(oedge<V,2> *oedges, int delta, V e, V v0, V v1)
{
	oedges[v0].overts[0] ^= v1;
	oedges[v0].degree += delta;
	oedges[v0].edge ^= e;
}

template<class V>
inline void
add_remove_oedge(oedge<V,3> *oedges, int delta, V e, V v0, V v1, V v2)
{
	oedges[v0].overts[v1 < v2 ? 0 : 1] ^= v1;
	oedges[v0].overts[v1 < v2 ? 1 : 0] ^= v2;
	oedges[v0].degree += delta;
	oedges[v0].edge ^= e;
}

template<class V>
inline void
remove_oedge(oedge<V,2> *oedges, V e, V v0, V v1)
{
	return add_remove_oedge(oedges, -1, e, v0, v1);
}

template<class V>
inline void
remove_oedge(oedge<V,3> *oedges, V e, V v0, V v1, V v2)
{
	return add_remove_oedge(oedges, -1, e, v0, v1, v2);
}

template<class V>
inline void
add_edge(oedge<V,2> *oedges, V e, V const *verts)
{
	add_remove_oedge(oedges, 1, e, verts[0], verts[1]);
	add_remove_oedge(oedges, 1, e, verts[1], verts[0]);
}

template<class V>
inline void
add_edge(oedge<V,3> *oedges, V e, V const *verts)
{
	add_remove_oedge(oedges, 1, e, verts[0], verts[1], verts[2]);
	add_remove_oedge(oedges, 1, e, verts[1], verts[0], verts[2]);
	add_remove_oedge(oedges, 1, e, verts[2], verts[0], verts[1]);
}

template<class V>
inline size_t
remove_vertex(oedge<V,2> *oedges, V v0, V *order, size_t top)
{
	if (oedges[v0].degree == 1) {
		V const e = oedges[v0].edge;
		V const v1 = oedges[v0].overts[0];
		oedges[v0].degree = 0;
		remove_oedge(oedges, e, v1, v0);
		order[--top] = e;
	}

	return top;
}

template<class V>
inline size_t
remove_vertex(oedge<V,3> *oedges, V v0, V *order, size_t top)
{
	if (oedges[v0].degree == 1) {
		V const e = oedges[v0].edge;
		V const v1 = oedges[v0].overts[0];
		V const v2 = oedges[v0].overts[1];
		oedges[v0].degree = 0;
		remove_oedge(oedges, e, v1, v0, v2);
		remove_oedge(oedges, e, v2, v0, v1);
		order[--top] = e;
	}

	return top;
}

// fast_divide32(3) from NetBSD.
inline uint32_t
fastdiv(uint32_t val, uint64_t mul, uint8_t s2)
{
	uint8_t constexpr s1 = 1; // s1 is 1 for every partsz greater than 1.
	uint32_t const hi = (val * mul) >> 32;

	return (hi + ((val - hi) >> s1)) >> s2;
}

// fast_remainder32(3) from NetBSD.
inline uint32_t
fastrem(uint32_t val, uint32_t div, uint64_t mul, uint8_t s2)
{
	return val - div * fastdiv(val, mul, s2);
}

template<int R>
fastrem_reduce<R>::fastrem_reduce(size_t nverts)
	: partsz(nverts / R)
{
	bool constexpr branchless = true;
	uint8_t s1;

	assert(partsz > 1 && (nverts % R) == 0);
	xorpeel_fastdiv_prepare(partsz, &mul, &s1, &s2, branchless);
	assert(s1 == 1); // s1 is 1 for every partsz greater than 1.
}

template<int R>
void
fastrem_reduce<R>::operator()(vert_t h[]) const
{
	for (int r = 0; r < R; r++)
		h[r] = fastrem(h[r], partsz, mul, s2) + r * partsz;
}

template<int R>
lemire_reduce<R>::lemire_reduce(size_t nverts)
	: partsz(nverts / R)
{
	assert(partsz > 1 && (nverts % R) == 0);
}

template<int R>
void
lemire_reduce<R>::operator()(vert_t h[]) const
{
	for (int r = 0; r < R; r++)
		h[r] = ((h[r] * (uint64_t)partsz) >> 32) + r * partsz;
}

template<int R>
custom_reduce<R>::custom_reduce(size_t nverts, xorpeel_reduce_t reduce)
	: partsz(nverts / R)
	, reduce(reduce)
{
	assert(partsz > 1 && (nverts % R) == 0);
}

template<int R>
void
custom_reduce<R>::operator()(vert_t h[]) const
{
	reduce(partsz, h);
	// XXX Check ranges.
}

template<class Reduce, class V, int R>
inline V
init_graph(xorpeel_iterator_t iter, void *state,
    xorpeel_hash_t hash, xorpeel_seed_t seed, Reduce const &reduce,
    edge<V,R> *edges, size_t nkeys, oedge<V,R> *oedges)
{
	V e = 0;
	void const *cur = iter(state);

	for (; e < nkeys && cur != nullptr; ++e, cur = iter(state)) {
		hash(cur, seed, edges[e].verts);
		reduce(edges[e].verts);
		add_edge(oedges, e, edges[e].verts);
	}

	return e;
}

template<class V, int R>
size_t
peel_graph(edge<V,R> const *edges, size_t nkeys,
    oedge<V,R> *oedges, size_t nverts, V *order)
{
	size_t top = nkeys;

	for (V v0 = 0; v0 < nverts; ++v0)
		top = remove_vertex(oedges, v0, order, top);

	for (size_t i = nkeys; i > 0 && i > top; --i) {
		edge<V,R> const &e = edges[order[i-1]];
		for (size_t r = 0; r < R; ++r)
			top = remove_vertex(oedges, e.verts[r], order, top);
	}

	return top;
}

inline size_t
edge_size(int rank)
{
	switch (rank) {
	case 2: return sizeof(edge<vert_t,2>);
	case 3: return sizeof(edge<vert_t,3>);
	default: return 0;
	}
}

inline size_t
oedge_size(int rank)
{
	switch (rank) {
	case 2: return sizeof(oedge<vert_t,2>);
	case 3: return sizeof(oedge<vert_t,3>);
	default: return 0;
	}
}

inline size_t constexpr
maxsize(size_t a, size_t b)
{
	return a > b ? a : b;
}

inline int
graph_rank(unsigned int flags)
{
	return (flags & XORPEEL_RANK_MASK) == XORPEEL_RANK2 ? 2 : 3;
}

inline size_t
graph_max_keys(unsigned int flags)
{
	int const rank = graph_rank(flags);

	return (rank == 2) ? MAX_NKEYS_R2_VEC : MAX_NKEYS_R3_VEC;
}

inline size_t
graph_nverts(int *flags, size_t nkeys)
{
	size_t const max_nkeys = graph_max_keys(*flags);

	if (nkeys == 0 || nkeys > max_nkeys)
		return 0;

	int const rank = graph_rank(*flags);
	// nverts is approx. 2.125*nkeys for rank 2 and 1.25*nkeys for rank 3.
	int const scale = 4 - rank;
	int const fract = 1 << (5 - rank);
	size_t const nv = scale * nkeys + round_up(nkeys, fract) / fract;
	size_t const nverts = maxsize(round_up(nv, rank), 24);

	assert(nverts > nkeys && nverts > scale * nkeys); // No overflow.

	return nverts;
}

template<class V, int R>
int
build_graph(struct xorpeel_graph *g, xorpeel_iterator_t iter,
    void *state, xorpeel_hash_t hash, xorpeel_seed_t seed,
    xorpeel_reduce_t reduce)
{
	typedef edge<V,R> edge_t;
	typedef oedge<V,R> oedge_t;

	auto order = static_cast<V *>(g->order);
	auto edges = static_cast<edge_t *>(g->edges);
	auto oedges = static_cast<oedge_t *>(g->oedges);
	size_t const nkeys = g->nkeys;
	size_t const nverts = g->nverts;
	unsigned int const flags = g->flags;
	int res = XORPEEL_INVAL;

	if ((flags & ZEROED) == 0) {
		memset(order, 0, sizeof(V) * nkeys);
		memset(edges, 0, sizeof(edge_t) * nkeys);
		memset(oedges, 0, sizeof(oedge_t) * nverts);
	}

	g->core_size = nkeys;
	g->seed = seed;
	g->flags &= PUBLIC_FLAGS; // Reset internal flags.

	switch (flags & XORPEEL_REDUCE_MASK) {
	case XORPEEL_REDUCE_MOD:
		res = init_graph(iter, state, hash, seed,
		    fastrem_reduce<R>(nverts), edges, nkeys, oedges);
		break;
	case XORPEEL_REDUCE_MUL:
		res = init_graph(iter, state, hash, seed,
		    lemire_reduce<R>(nverts), edges, nkeys, oedges);
		break;
	default:
	case XORPEEL_REDUCE_CUSTOM:
		res = init_graph(iter, state, hash, seed,
		    custom_reduce<R>(nverts, reduce), edges, nkeys, oedges);
		break;
		assert(0 && "xorpeel_alloc_graph() should have caught it");
		return XORPEEL_INVAL;
	}

	if (res != XORPEEL_SUCCESS)
		return res;

	g->core_size = peel_graph(edges, nkeys, oedges, nverts, order);

	g->flags |= BUILT;
	return g->core_size == 0 ? XORPEEL_SUCCESS : XORPEEL_AGAIN;
}

} // anon namespace

extern "C"
void
xorpeel_free_graph(struct xorpeel_graph *g)
{
	if (g != nullptr) {
		free(g->oedges);
		free(g->edges);
		free(g->order);
		free(g);
	}
}

extern "C"
struct xorpeel_graph *
xorpeel_alloc_graph(size_t nkeys, int flags)
{
	struct xorpeel_graph *g;
	size_t nverts, esz, osz;
	int save_errno;
	int r;

	nverts = graph_nverts(&flags, nkeys);
	if (nverts == 0) {
		errno = ERANGE;
		return nullptr;
	}

	assert(nverts > nkeys);

	r = graph_rank(flags);
	esz = edge_size(r);
	if (esz == 0) {
		errno = EINVAL;
		return nullptr;
	}

	osz = oedge_size(r);
	if (osz == 0) {
		errno = ENOMEM;
		return nullptr;
	}

	g = static_cast<struct xorpeel_graph *>(calloc(sizeof(*g), 1));
	if (g == nullptr)
		return nullptr;

	g->order    = nullptr;
	g->edges    = nullptr;
	g->oedges   = nullptr;

	g->order = calloc(sizeof(vert_t), nkeys);
	if (g->order == nullptr)
		goto err;

	g->edges = calloc(esz, nkeys);
	if (g->edges == nullptr)
		goto err;

	g->oedges = calloc(osz, nverts);
	if (g->oedges == nullptr)
		goto err;

	g->seed       = 0;
	g->nkeys      = nkeys;
	g->nverts     = nverts;
	g->core_size  = nkeys;
	g->flags      = flags | ZEROED; // calloc

	return g;
err:
	save_errno = errno;
	xorpeel_free_graph(g);
	errno = save_errno;
	return nullptr;
}

extern "C"
int
xorpeel_flags(struct xorpeel_graph const *g)
{
	return g->flags & PUBLIC_FLAGS;
}

extern "C"
int
xorpeel_rank(struct xorpeel_graph const *g)
{
	return graph_rank(g->flags);
}

extern "C"
size_t
xorpeel_num_entries(struct xorpeel_graph const *g)
{
	return g->nkeys;
}

extern "C"
size_t
xorpeel_num_vertices(struct xorpeel_graph const *g)
{
	return g->nverts;
}

extern "C"
size_t
xorpeel_core_size(struct xorpeel_graph const *g)
{
	return g->core_size;
}

extern "C"
xorpeel_seed_t
xorpeel_seed(struct xorpeel_graph const *g)
{
	return g->seed;
}

extern "C"
int
xorpeel_is_built(struct xorpeel_graph const *g)
{
	return (g->flags & BUILT) != 0;
}

extern "C"
int
xorpeel_is_assigned(struct xorpeel_graph const *g)
{
	int const res = (g->flags & ASSIGNED) != 0;

	assert(res != ((g->flags & PEELED) != 0));
	return res;
}

extern "C"
int
xorpeel_build_graph(struct xorpeel_graph *g, xorpeel_hash_t hash,
    xorpeel_seed_t seed, xorpeel_iterator_t iter, void *state,
    xorpeel_reduce_t reduce)
{
	int rank = graph_rank(g->flags);

	switch (rank) {
	case 2:
		return build_graph<vert_t,2>(g, iter, state, hash, seed, reduce);
	case 3:
		return build_graph<vert_t,3>(g, iter, state, hash, seed, reduce);
	default:
		assert(0 && "xorpeel_alloc_graph() should have caught it");
		return XORPEEL_INVAL;
	}
}

extern "C"
size_t
xorpeel_count_keys(xorpeel_iterator_t iter, void *state)
{
	size_t res = 0;

	for (void const *cur = iter(state); cur != nullptr; cur = iter(state))
		res++;
	return res;
}
