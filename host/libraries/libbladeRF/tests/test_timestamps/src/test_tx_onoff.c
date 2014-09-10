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
    unsigned int buf_len;
    unsigned int burst_len;     /* Length of a burst, in samples */
    unsigned int gap_len;       /* Gap between bursts, in samples */
    unsigned int iterations;
};

static const struct test_case tests[] = {
    { 1024, 1006,   2000, 8 },
#if 0
    { 1024, 100000, 50000,  10 },
    { 1024, 100000, 25000,  10 },
    { 1024, 100000, 1000,   10 },

    { 1024, 100000, 10000,  10 },
    { 1024, 50000,  10000,  10 },
    { 1024, 25000,  10000,  10 },
    { 1024, 1000,   10000,  10 },
#endif
};

static int run(struct bladerf *dev, struct app_params *p,
               int16_t *buf, const struct test_case *t)
{
    int status, status_out;
    unsigned int samples_left;
    size_t i;
    struct bladerf_metadata meta;


    memset(&meta, 0, sizeof(meta));

    status = perform_sync_init(dev, BLADERF_MODULE_TX, t->buf_len, p);
    if (status != 0) {
        goto out;
    }

    status = bladerf_get_timestamp(dev, BLADERF_MODULE_TX, &meta.timestamp);
    if (status != 0) {
        fprintf(stderr, "Failed to get timestamp: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("Initial timestamp: 0x%016"PRIx64"\n", meta.timestamp);
    }

    meta.timestamp += 200000;


    for (i = 0; i < t->iterations && status == 0; i++) {
        meta.flags = BLADERF_META_FLAG_TX_BURST_START;
        samples_left = t->burst_len;

        printf("Sending burst @ %llu\n", (unsigned long long) meta.timestamp);

        while (samples_left && status == 0) {
            unsigned int to_send = uint_min(p->buf_size, samples_left);

            if (to_send == samples_left) {
                meta.flags |= BLADERF_META_FLAG_TX_BURST_END;
            } else {
                meta.flags &= ~BLADERF_META_FLAG_TX_BURST_END;
            }

            status = bladerf_sync_tx(dev, buf, to_send, &meta, 10000); //p->timeout_ms);
            if (status != 0) {
                fprintf(stderr, "TX failed @ iteration (%u) %s\n",
                        (unsigned int )i, bladerf_strerror(status));
            }

            meta.flags &= ~BLADERF_META_FLAG_TX_BURST_START;
            samples_left -= to_send;
        }

        meta.timestamp += (t->burst_len + t->gap_len);
    }

    printf("Waiting for samples to finish...\n");

    /* Wait for samples to be transmitted before shutting down the TX module */
    // FIXME get_timestamp doesn't work
#if 0
    timestamp_now = 0;
    while (status == 0 && timestamp_now < meta.timestamp) {
        status = bladerf_get_timestamp(dev, BLADERF_MODULE_TX, &timestamp_now);
        usleep(1000000);
        printf("Waiting for TX to be done: 0x%"PRIx64" 0x%"PRIx64"\n",
                meta.timestamp, timestamp_now);
    }
#else
    usleep(2000000);
#endif

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
        assert((tests[i].burst_len + tests[i].gap_len) >= tests[i].buf_len);
        status = run(dev, p, samples, &tests[i]);
    }

    free(samples);
    return status;
}


