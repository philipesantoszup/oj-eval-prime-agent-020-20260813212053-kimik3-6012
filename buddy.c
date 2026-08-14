#include "buddy.h"
#include <stdlib.h>
#include <stdint.h>

#define MAXRANK 16
#define PGSZ 4096UL

/* Global allocator state */
static char *pool_base = NULL;
static long npages = 0;

/* blk_rank[i] != 0  <=> page i is the start of a buddy block.
 *   > 0 : the block is free,  value = rank
 *   < 0 : the block is allocated, value = -rank
 * Since buddy blocks tile the whole pool, every page belongs to exactly
 * one block, whose start is the nearest preceding page with blk_rank != 0.
 */
static int *blk_rank = NULL;

/* Doubly linked free lists (indexed by page index of block starts) */
static long *fl_next = NULL;
static long *fl_prev = NULL;
static long fl_head[MAXRANK + 1];
static long free_cnt[MAXRANK + 1];

static void list_remove(long idx, int rank) {
    long p = fl_prev[idx], n = fl_next[idx];
    if (p >= 0) fl_next[p] = n; else fl_head[rank] = n;
    if (n >= 0) fl_prev[n] = p;
    fl_prev[idx] = fl_next[idx] = -1;
    free_cnt[rank]--;
}

static void list_push_head(long idx, int rank) {
    fl_prev[idx] = -1;
    fl_next[idx] = fl_head[rank];
    if (fl_head[rank] >= 0) fl_prev[fl_head[rank]] = idx;
    fl_head[rank] = idx;
    free_cnt[rank]++;
}

static void list_push_tail(long idx, int rank) {
    if (fl_head[rank] < 0) {
        list_push_head(idx, rank);
        return;
    }
    long t = fl_head[rank];
    while (fl_next[t] >= 0) t = fl_next[t];
    fl_next[t] = idx;
    fl_prev[idx] = t;
    fl_next[idx] = -1;
    free_cnt[rank]++;
}

int init_page(void *p, int pgcount) {
    if (p == NULL || pgcount <= 0) return -EINVAL;

    free(blk_rank); free(fl_next); free(fl_prev);
    blk_rank = NULL; fl_next = NULL; fl_prev = NULL;
    pool_base = NULL; npages = 0;

    blk_rank = (int *)calloc((size_t)pgcount, sizeof(int));
    fl_next  = (long *)malloc((size_t)pgcount * sizeof(long));
    fl_prev  = (long *)malloc((size_t)pgcount * sizeof(long));
    if (!blk_rank || !fl_next || !fl_prev) {
        free(blk_rank); free(fl_next); free(fl_prev);
        blk_rank = NULL; fl_next = NULL; fl_prev = NULL;
        return -ENOSPC;
    }

    pool_base = (char *)p;
    npages = pgcount;
    for (int r = 0; r <= MAXRANK; r++) { fl_head[r] = -1; free_cnt[r] = 0; }

    /* Binary decomposition of pgcount into aligned buddy blocks,
     * largest first, so every block start is naturally aligned. */
    long i = 0, remaining = pgcount;
    for (int r = MAXRANK; r >= 1 && remaining > 0; r--) {
        long sz = 1L << (r - 1);
        while (remaining >= sz) {
            blk_rank[i] = r;
            list_push_tail(i, r);   /* keep ascending address order */
            i += sz;
            remaining -= sz;
        }
    }
    return OK;
}

void *alloc_pages(int rank) {
    if (rank < 1 || rank > MAXRANK) return ERR_PTR(-EINVAL);
    if (pool_base == NULL) return ERR_PTR(-ENOSPC);

    int r = rank;
    while (r <= MAXRANK && fl_head[r] < 0) r++;
    if (r > MAXRANK) return ERR_PTR(-ENOSPC);

    long idx = fl_head[r];
    list_remove(idx, r);

    /* Split down; push the HIGH half of each split onto the free list so
     * that the LOW half keeps being used -> ascending addresses. */
    while (r > rank) {
        r--;
        long hi = idx + (1L << (r - 1));
        blk_rank[hi] = r;
        list_push_head(hi, r);
        blk_rank[idx] = r;
    }
    blk_rank[idx] = -rank;  /* mark allocated */
    return (void *)(pool_base + (uintptr_t)idx * PGSZ);
}

static long page_index_of(void *p) {
    if (p == NULL || pool_base == NULL) return -1;
    uintptr_t addr = (uintptr_t)p, b = (uintptr_t)pool_base;
    if (addr < b) return -1;
    uintptr_t off = addr - b;
    if (off % PGSZ != 0) return -1;
    long idx = (long)(off / PGSZ);
    if (idx >= npages) return -1;
    return idx;
}

int return_pages(void *p) {
    long idx = page_index_of(p);
    if (idx < 0) return -EINVAL;
    if (blk_rank[idx] >= 0) return -EINVAL;  /* free block or mid-block */

    int rank = -blk_rank[idx];
    blk_rank[idx] = rank;

    /* Coalesce with buddy while possible */
    while (rank < MAXRANK) {
        long buddy = idx ^ (1L << (rank - 1));
        if (buddy >= npages || blk_rank[buddy] != rank) break;
        list_remove(buddy, rank);
        blk_rank[idx] = 0;
        blk_rank[buddy] = 0;
        if (buddy < idx) idx = buddy;
        rank++;
        blk_rank[idx] = rank;
    }
    list_push_head(idx, rank);
    return OK;
}

int query_ranks(void *p) {
    long idx = page_index_of(p);
    if (idx < 0) return -EINVAL;
    if (blk_rank[idx] != 0) {
        int r = blk_rank[idx];
        return r > 0 ? r : -r;
    }
    /* mid-block page: nearest preceding block start contains it */
    long j = idx - 1;
    while (j >= 0 && blk_rank[j] == 0) j--;
    if (j < 0) return -EINVAL;
    int r = blk_rank[j];
    return r > 0 ? r : -r;
}

int query_page_counts(int rank) {
    if (rank < 1 || rank > MAXRANK) return -EINVAL;
    return (int)free_cnt[rank];
}
