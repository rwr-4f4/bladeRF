/*
 * This test excercises functionatity to wait for specific timestamps
 *
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2014 Nuand LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include <libbladeRF.h>
#include "test_timestamps.h"
#include "rel_assert.h"

struct test_case {
    uint64_t gap;
    unsigned int read_size;
    unsigned int iterations;
};

static const struct test_case tests[] = {
    /* Gap          Read Size       Iterations */
    { 2 * 1024,         1,              1000 },
    { 2 * 1024,         1024,           1000 },
};

static int run(struct bladerf *dev, struct app_params *p,
               int16_t *samples, const struct test_case *t)
{
    int status;
    struct bladerf_metadata meta;
    unsigned int i;
    bool pass = true;

    memset(&meta, 0, sizeof(meta));

    status = bladerf_sync_config(dev,
                                 BLADERF_MODULE_RX,
                                 BLADERF_FORMAT_SC16_Q11_META,
                                 p->num_buffers,
                                 p->buf_size,
                                 p->num_xfers,
                                 p->timeout_ms);

    if (status != 0) {
        fprintf(stderr, "Failed to configure RX sync i/f: %s\n",
                bladerf_strerror(status));
        return status;
    }

    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, true);
    if (status != 0) {
        fprintf(stderr, "Failed to enable RX module: %s\n",
                bladerf_strerror(status));

        goto out;
    }

    printf("\nTest case: Gap=%"PRIu64" samples, Read size=%u, %u iterations\n",
           t->gap, t->read_size, t->iterations);
    printf("--------------------------------------------------------\n");

    for (i = 0; i < t->iterations && status == 0 && pass; i++) {
        assert(t->gap >= t->read_size);

        meta.timestamp += t->gap;

        status = bladerf_sync_rx(dev, samples, t->read_size,
                                 &meta, p->timeout_ms);

        if (status != 0) {
            fprintf(stderr, "RX %u failed: %s\n", i, bladerf_strerror(status));
            pass = false;
        }
    }

    printf("Test %s.\n", pass ? "passed" : "failed");

out:
    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
    if (status != 0) {
        fprintf(stderr, "Failed to disable RX module: %s\n",
                bladerf_strerror(status));
    }

    return status;
}

int test_fn_rx_scheduled(struct bladerf *dev, struct app_params *p)
{
    int status = 0;
    int16_t *samples;
    size_t i;

    samples = malloc(p->buf_size * 2 * sizeof(int16_t));
    if (samples == NULL) {
        perror("malloc");
        return BLADERF_ERR_MEM;
    }

    for (i = 0; i < ARRAY_SIZE(tests) && status == 0; i++) {
        status = run(dev, p, samples, &tests[i]);
    }

    free(samples);
    return status;
}
