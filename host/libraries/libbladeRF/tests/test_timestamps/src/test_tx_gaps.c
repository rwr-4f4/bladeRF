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
#include <inttypes.h>
#include <libbladeRF.h>
#include "test_timestamps.h"

/* This test requires external verification via a spectrum analyzer.
 * It simply transmits ON/OFF bursts */

#define MAGNITUDE 2000

struct test_case {
    uint64_t samples_on;
    uint64_t samples_off;
    unsigned int iterations;
};

struct test_info {
    uint64_t on_left;
    uint64_t off_left;
    size_t buf_left;

    enum {
        VAL_RESET,
        VAL_ON,
        VAL_OFF,
    } val_state;

    enum buf_state {
        BUF_RESET,
        BUF_FILLING,
        BUF_FULL,
    } buf_state;
}

static const struct test_case tests[] = {
    { 100000, 100000, 10 },
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
    int status;
    const size_t buf_size = DEFAULT_BUF_SIZE;
    struct test_info t;
    unsigned int i;

    t.val_state = VAL_RESET;
    t.buf_state = BUF_RESET;

    for (i = 0; i < iterations; i++) {
        switch (t.buf_state) {
            case BUF_RESET:
                memset(buf, 0, 2 * buf_size * sizeof(buf[0]));
                t.buf_left = buf_size;
                t.state = BUF_FILLING;
                break;

            case BUF_FILLING:
                break;

            case BUF_FULL:
                status = bladerf_sync_tx(dev, samples, buf_size,
                t.state = BUF_RESET;
                continue;
        };


        switch (t.state) {
            case VAL_RESET:
                t.on_left = 0;
                t.state = VAL_ON;
                break;

            case VAL_ON:
                break;

            case VAL_OFF:
                break;

            default:
                assert(!"Invalid state!");
        }

    }
}

int test_fn_tx_gaps(struct bladerf *dev, struct app_params *p)
{
    int status = 0;
    int16_t *samples;
    size_t i;

    samples = malloc(BUF_SIZE * 2 * sizeof(int16_t));
    if (samples == NULL) {
        perror("malloc");
        return BLADERF_ERR_MEM;
    }

    for (i = 0; i < num_tests && status == 0; i++) {
        status = run(dev, p, samples, &tests[i]);
    }

    free(samples);
    return status;
}


