/*
 * This test sends some GMSK bursts. The output should be verified with a
 * spectrum analyze, preferably one able quantify phase and freq error.
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
#include <stdint.h>
#include <limits.h>
#include <inttypes.h>
#include <libbladeRF.h>
#include "test_timestamps.h"
#include "gmsk_burst.h"

static const struct test_case {
    unsigned int buf_len;
    unsigned int iterations;
} tests[] = {
    { 2048, 1000 },
};

static int run(struct bladerf *dev, struct app_params *p,
               const struct test_case *t)
{
    int status, status_out;
    struct bladerf_metadata meta;
    unsigned int i;
    const unsigned int burst_len = (unsigned int) ARRAY_SIZE(gmsk_burst) / 2;

    memset(&meta, 0, sizeof(meta));
    meta.flags = BLADERF_META_FLAG_TX_BURST_START |
                 BLADERF_META_FLAG_TX_BURST_END;

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

    /* Start 1M samples in */
    meta.timestamp += 1000000;

    for (i = status = 0; i < t->iterations && status == 0; i++) {
        status = bladerf_sync_tx(dev, gmsk_burst, burst_len, &meta, 120000);

        /* Distance between beginning of each burst is 100k samples */
        meta.timestamp += 100000 - burst_len;
    }

    usleep(1000000);

out:
    status_out = bladerf_enable_module(dev, BLADERF_MODULE_TX, false);
    status = first_error(status, status_out);

    return status;
}

int test_fn_tx_gmsk_bursts(struct bladerf *dev, struct app_params *p)
{
    size_t i;
    int status, status_restore;
    unsigned int samplerate_backup;
    struct bladerf_rational_rate samplerate = GMSK_SAMPLERATE_INITIALIZER;

    status = bladerf_get_sample_rate(dev, BLADERF_MODULE_TX,
                                     &samplerate_backup);

    if (status != 0) {
        fprintf(stderr, "Failed to read TX samplerate: %s\n",
                bladerf_strerror(status));
        return -1;
    }

    status = bladerf_set_rational_sample_rate(dev, BLADERF_MODULE_TX,
                                              &samplerate, NULL);

    if (status != 0) {
        fprintf(stderr, "Failed to read TX samplerate: %s\n",
                bladerf_strerror(status));
        return -1;
    }

    for (i = status = 0; i < ARRAY_SIZE(tests) && status == 0; i++) {
        status = run(dev, p, &tests[i]);
    }

    status_restore  = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX,
                                              samplerate_backup, NULL);
    if (status_restore != 0) {
        fprintf(stderr, "Failed to restore TX samplerate: %s\n",
                bladerf_strerror(status));
    }

    return status;
}
