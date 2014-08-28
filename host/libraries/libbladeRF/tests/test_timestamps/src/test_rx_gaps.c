/*
 * This test simply receives any available samples and checks that there are no
 * gaps/jumps in the expected timestamp
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

#define RANDOM_GAP_SIZE 0

struct test_case {
    uint64_t gap;
    unsigned int iterations;
};

static const struct test_case tests[] = {
#if 0
    { 1,    2000000 },
    { 2,    2000000 },
    { 128,  75000 },
    { 256,  50000 },
    { 512,  50000 },
    { 1023, 10000 },
#endif
    { 1024, 10000 },

#if 0
    { 1025, 10000 },
    { 2048, 5000 },
    { 3172, 5000 },
    { 4096, 2500 },
    { 8192, 2500 },
    { 16 * 1024, 1000 },
    { 32 * 1024, 1000 },
    { 64 * 1024, 1000 },
    { RANDOM_GAP_SIZE, 500 },
#endif
};

static inline uint64_t get_gap(struct app_params *p, const struct test_case *t)
{
    uint64_t gap;

    if (t->gap == 0) {
        const uint64_t tmp = randval_update(&p->prng_state) % p->buf_size;
        if (tmp == 0) {
            gap = p->buf_size;
        } else {
            gap = tmp;
        }
    } else {
        gap = t->gap;
    }

    assert(gap <= p->buf_size);
    return gap;
}

static int run(struct bladerf *dev, struct app_params *p,
               int16_t *samples, const struct test_case *t)
{
    int status, status_out;
    struct bladerf_metadata meta;
    uint64_t timestamp, gap;
    uint32_t counter;
    unsigned int i;
    bool pass = true;

    /* Clear out metadata and request that we just received any available
     * samples, with the timestamp filled in for us */
    memset(&meta, 0, sizeof(meta));
    meta.flags = BLADERF_META_FLAG_NOW;

    status = perform_sync_init(dev, BLADERF_MODULE_RX, p);
    if (status != 0) {
        goto out;
    }

    status = enable_counter_mode(dev, true);
    if (status != 0) {
        goto out;
    }

    if (t->gap != 0) {
        printf("\nTest Case: Read size=%"PRIu64" samples, %u iterations\n",
                t->gap, t->iterations);
    } else {
        printf("\nTest Case: Random read size, %u iterations\n", t->iterations);
    }
    printf("--------------------------------------------------------\n");

    /* Initial read to get a starting timestamp, and counter value */
    gap = get_gap(p, t);
    status = bladerf_sync_rx(dev, samples, gap, &meta, p->timeout_ms);
    if (status != 0) {
        fprintf(stderr, "Intial RX failed: %s\n", bladerf_strerror(status));
        goto out;
    }

    counter = extract_counter_val(samples);
    if (!counter_data_is_valid(samples, gap, &counter)) {
        pass = false;
    }

    printf("Initial timestamp:      0x%016"PRIx64"\n", meta.timestamp);
    printf("Intital counter value:  0x%08"PRIx32"\n", counter);
    printf("Initial status:         0x%08"PRIu32"\n", meta.status);

    for (i = 0; i < t->iterations && status == 0 && pass; i++) {

        timestamp = meta.timestamp + gap;
        gap = get_gap(p, t);

        status = bladerf_sync_rx(dev, samples, gap, &meta, p->timeout_ms);
        if (status != 0) {
            fprintf(stderr, "RX %u failed: %s\n", i, bladerf_strerror(status));
            goto out;
        }

        if (meta.timestamp != timestamp) {
            pass = false;
            fprintf(stderr, "Timestamp mismatch @ %u. "
                    "Expected 0x%016"PRIx64", got 0x%016"PRIx64"\n",
                    i, timestamp, meta.timestamp);

        }

        if (meta.status != 0) {
            pass = false;
            fprintf(stderr, "Metadata status: 0x%08"PRIu32"\n", meta.status);
        }

        if (!counter_data_is_valid(samples, gap, &counter)) {
            pass = false;
        }
    }

    printf("Test %s.\n", pass ? "passed" : "failed");

out:
    status_out = bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
    if (status_out != 0) {
        fprintf(stderr, "Failed to disable RX module: %s\n",
                bladerf_strerror(status));
    }

    status = first_error(status, status_out);

    status_out = enable_counter_mode(dev, false);
    status = first_error(status, status_out);

    return status;
}

int test_fn_rx_gaps(struct bladerf *dev, struct app_params *p)
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
