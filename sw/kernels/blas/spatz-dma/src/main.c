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

#define snrt_dma_start_1d spatz_dma_start_1d
#define snrt_dma_wait_all spatz_dma_wait_all

int main() {
    double *local_a = NULL;
    double *local_b = NULL;
    if (snrt_is_dm_core()) {
        local_a = (double *)snrt_l1_alloc(n * sizeof(double));
        local_b = (double *)snrt_l1_alloc(n * sizeof(double));
    }

    if (snrt_is_dm_core()) {
        unsigned int t0 = snrt_mcycle();
        snrt_dma_start_1d(local_a, a, n * sizeof(double));
        snrt_dma_start_1d(local_b, b, n * sizeof(double));
        snrt_dma_wait_all();
        printf("DMA load cycles: %u\r\n", snrt_mcycle() - t0);
    }

    snrt_cluster_hw_barrier();

    if (snrt_is_dm_core()) {
        unsigned int t0 = snrt_mcycle();
        snrt_dma_start_1d(a, local_a, n * sizeof(double));
        snrt_dma_start_1d(b, local_b, n * sizeof(double));
        snrt_dma_wait_all();
        printf("DMA store cycles: %u\r\n", snrt_mcycle() - t0);
    }

    snrt_cluster_hw_barrier();

    return 0;
}
#endif
