/*
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
#include <stdint.h>
#include <limits.h>
#include <inttypes.h>
#include <libbladeRF.h>
#include "test_timestamps.h"
#include "minmax.h"

/* This test requires external verification via a spectrum analyzer.
 * It simply transmits ON/OFF bursts, and is more intended to ensure the API
 * functions aren't bombing out than it is to exercise signal integrity/timing.
 */

#define MAGNITUDE 2000

struct test_case {
    unsigned int usec_on;
    unsigned int usec_off;
    unsigned int iterations;
};

static const struct test_case tests[] = {
    { 1000, 100000, 1 }
#if 0
    { 100000, 50000,  10},
    { 100000, 25000,  10 },
    { 100000, 1000,   10 },

    { 100000, 10000,  10 },
    { 50000,  10000,  10 },
    { 25000,  10000,  10 },
    { 1000,   10000,  10 },
#endif
};

static int run(struct bladerf *dev, struct app_params *p,
               int16_t *buf, const struct test_case *t)
{
    int status, status_out;
    unsigned int samples_left;
    uint64_t timestamp, samples_on, samples_off;
    size_t i;
    struct bladerf_metadata meta;

    if (p->samplerate < 1000000) {
        fprintf(stderr, "Sample rate is too low for this test.\n");
        return -1;
    }

    samples_on  = t->usec_on * (p->samplerate / 1000000);
    samples_off = t->usec_off * (p->samplerate / 1000000);

    memset(&meta, 0, sizeof(meta));

    /* Try to keep these within 32-bit boundaries, please */
    assert(samples_on <= UINT_MAX);
    assert(samples_off <= UINT_MAX);

    status = perform_sync_init(dev, BLADERF_MODULE_TX, p);
    if (status != 0) {
        goto out;
    }

    status = bladerf_get_timestamp(dev, BLADERF_MODULE_TX, &timestamp);
    if (status != 0) {
        fprintf(stderr, "Failed to get timestamp: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("Initial timestamp: 0x%016"PRIx64"\n", timestamp);
    }

    timestamp += 200000;


    for (i = 0; i < t->iterations && status == 0; i++) {
        meta.timestamp = timestamp;
        meta.flags = BLADERF_META_FLAG_TX_BURST_START;
        samples_left = samples_on;

        while (samples_left && status == 0) {
            unsigned int to_send = uint_min(p->buf_size, samples_left);
            if (to_send == samples_left) {
                meta.flags |= BLADERF_META_FLAG_TX_BURST_END;
            } else {
                meta.flags &= ~BLADERF_META_FLAG_TX_BURST_END;
            }

            status = bladerf_sync_tx(dev, buf, to_send, &meta, p->timeout_ms);
            if (status != 0) {
                fprintf(stderr, "TX failed: %s\n", bladerf_strerror(status));
            }

            meta.flags &= ~BLADERF_META_FLAG_TX_BURST_START;
            samples_left -= to_send;
        }

        timestamp += (samples_off + samples_on);
    }

out:
    status_out = bladerf_enable_module(dev, BLADERF_MODULE_TX, false);
    if (status_out != 0) {
        fprintf(stderr, "Failed to disable TX module: %s\n",
                bladerf_strerror(status));
    }

    status = first_error(status, status_out);

    return status;
}

int test_fn_tx_onoff(struct bladerf *dev, struct app_params *p)
{
    int status = 0;
    int16_t *samples;
    size_t i;

    samples = malloc(p->buf_size * 2 * sizeof(int16_t));
    if (samples == NULL) {
        perror("malloc");
        return BLADERF_ERR_MEM;
    }

    for (i = 0; i < (2 * p->buf_size); i += 2) {
        samples[i] = samples[i + 1] = MAGNITUDE;
    }

    for (i = 0; i < ARRAY_SIZE(tests) && status == 0; i++) {
        status = run(dev, p, samples, &tests[i]);
    }

    free(samples);
    return status;
}


