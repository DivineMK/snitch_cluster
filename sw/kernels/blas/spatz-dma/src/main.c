// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <snrt.h>
#include <stdint.h>
#include <stdio.h>

#include "data.h"

#ifndef SNRT_SUPPORTS_VECTOR
int main() { return 0; }
#else

#define SPATZ_DMA_TEST_COPY 1
#define SPATZ_DMA_TEST_MICRO 0

static inline void spatz_dma_wait_all(const uint32_t ch = 0) {
    asm volatile("fence" ::: "memory");
}

static inline void spatz_dma_start_1d(volatile void *dst, volatile void *src,
                                     size_t size, uint32_t ch = 0) {
    unsigned int vl;
    unsigned int avl = size / sizeof(uint64_t);
    uint64_t *d = (uint64_t *)dst;
    uint64_t *s = (uint64_t *)src;
    do {
        asm volatile("vsetvli %0, %1, e64, m8, ta, ma" : "=r"(vl) : "r"(avl));
        asm volatile("vle64.v v0, (%0)" : : "r"(s));
        asm volatile("vse64.v v0, (%0)" : : "r"(d));
        s += vl;
        d += vl;
        avl -= vl;
    } while (avl > 0);
}

static void spatz_dma_copy(void) {
    double *local_a = NULL;
    double *local_b = NULL;
    if (snrt_is_dm_core()) {
        local_a = (double *)snrt_l1_alloc(n * sizeof(double));
        local_b = (double *)snrt_l1_alloc(n * sizeof(double));
    }

    if (snrt_is_dm_core()) {
        unsigned int t0 = snrt_mcycle();
        spatz_dma_start_1d(local_a, a, n * sizeof(double));
        spatz_dma_start_1d(local_b, b, n * sizeof(double));
        spatz_dma_wait_all();
        printf("DMA load cycles: %u\r\n", snrt_mcycle() - t0);
    }

    snrt_cluster_hw_barrier();

    if (snrt_is_dm_core()) {
        unsigned int t0 = snrt_mcycle();
        spatz_dma_start_1d(a, local_a, n * sizeof(double));
        spatz_dma_start_1d(b, local_b, n * sizeof(double));
        spatz_dma_wait_all();
        printf("DMA store cycles: %u\r\n", snrt_mcycle() - t0);
    }

    snrt_cluster_hw_barrier();

    const unsigned int boundary_offset = (4096 - 256) / sizeof(double);
    const unsigned int boundary_count = 64;
    if (snrt_is_dm_core()) {
        unsigned int t0 = snrt_mcycle();
        // Cross a 4 KiB boundary on both the AXI read and write paths.
        spatz_dma_start_1d(local_b + boundary_offset, a + boundary_offset,
                           boundary_count * sizeof(double));
        spatz_dma_wait_all();
        spatz_dma_start_1d(b + boundary_offset, local_b + boundary_offset,
                           boundary_count * sizeof(double));
        // Exercise a single 64 B wide read and write beat.
        spatz_dma_start_1d(local_b, a, 8 * sizeof(double));
        spatz_dma_wait_all();
        spatz_dma_start_1d(b, local_b, 8 * sizeof(double));
        spatz_dma_wait_all();
        printf("DMA boundary/single-beat cycles: %u\r\n",
               snrt_mcycle() - t0);
    }
}

#if SPATZ_DMA_TEST_MICRO
static inline void spatz_dma_load_only(const volatile void *src, size_t size) {
    unsigned int vl;
    unsigned int avl = size / sizeof(uint64_t);
    const uint64_t *s = (const uint64_t *)src;
    do {
        asm volatile("vsetvli %0, %1, e64, m8, ta, ma" : "=r"(vl) : "r"(avl));
        asm volatile("vle64.v v0, (%0)" : : "r"(s) : "memory");
        s += vl;
        avl -= vl;
    } while (avl > 0);
}

static inline void spatz_dma_store_only(volatile void *dst, size_t size) {
    unsigned int vl;
    unsigned int avl = size / sizeof(uint64_t);
    uint64_t *d = (uint64_t *)dst;
    do {
        asm volatile("vsetvli %0, %1, e64, m8, ta, ma" : "=r"(vl) : "r"(avl));
        asm volatile("vse64.v v0, (%0)" : : "r"(d) : "memory");
        d += vl;
        avl -= vl;
    } while (avl > 0);
}

static void spatz_dma_micro(void) {
    unsigned int t0;
    unsigned int vl;
    uint64_t *l1a = (uint64_t *)snrt_l1_alloc(n * sizeof(double));
    uint64_t *l1b = (uint64_t *)snrt_l1_alloc(n * sizeof(double));

    t0 = snrt_mcycle();
    spatz_dma_load_only(l1a, n * sizeof(double));
    spatz_dma_load_only(l1b, n * sizeof(double));
    spatz_dma_wait_all();
    printf("TCDM load-only cycles: %u\r\n", snrt_mcycle() - t0);

    t0 = snrt_mcycle();
    spatz_dma_store_only(l1a, n * sizeof(double));
    spatz_dma_store_only(l1b, n * sizeof(double));
    spatz_dma_wait_all();
    printf("TCDM store-only cycles: %u\r\n", snrt_mcycle() - t0);

    t0 = snrt_mcycle();
    spatz_dma_load_only(a, n * sizeof(double));
    spatz_dma_load_only(b, n * sizeof(double));
    snrt_dma_wait_all();
    printf("DMA load-only cycles: %u\r\n", snrt_mcycle() - t0);

    t0 = snrt_mcycle();
    spatz_dma_store_only(a, n * sizeof(double));
    spatz_dma_store_only(b, n * sizeof(double));
    snrt_dma_wait_all();
    printf("DMA store-only cycles: %u\r\n", snrt_mcycle() - t0);
}
#endif

// One main. The mode only selects which workload runs; the verified copy is the
// default so `just vsim-verify` always checks the real thing.
int main() {
#if SPATZ_DMA_TEST_MICRO
    if (snrt_is_dm_core()) spatz_dma_micro();
#else
    spatz_dma_copy();
#endif
    return 0;
}

#endif
