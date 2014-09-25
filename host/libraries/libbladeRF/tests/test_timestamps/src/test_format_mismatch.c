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

/* This test exercises some error-checking with repect to the RX and TX
 * modules being configured with conflicting formats */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <pthread.h>
#include <libbladeRF.h>
#include "test_timestamps.h"

const struct interfaces {
    bool rx_sync;
    bool tx_sync;
} ifs[] = {
    { true, true },
};

const struct test_case {
    bladerf_format rx_fmt;
    bladerf_format tx_fmt;
    int exp_status;
} tests[] = {
    { BLADERF_FORMAT_SC16_Q11, BLADERF_FORMAT_SC16_Q11, 0 },
    { BLADERF_FORMAT_SC16_Q11, BLADERF_FORMAT_SC16_Q11_META, BLADERF_ERR_INVAL },
    { BLADERF_FORMAT_SC16_Q11_META, BLADERF_FORMAT_SC16_Q11, BLADERF_ERR_INVAL  },
    { BLADERF_FORMAT_SC16_Q11_META, BLADERF_FORMAT_SC16_Q11_META, 0 },
};

static int enable_modules(struct bladerf *dev, bool enable)
{
    int status;

    status = bladerf_enable_module(dev, BLADERF_MODULE_RX, enable);
    if (status != 0) {
        fprintf(stderr, "Failed to %s RX module: %s\n",
                enable ? "enable" : "disable",
                bladerf_strerror(status));
        return status;
    }

    status = bladerf_enable_module(dev, BLADERF_MODULE_TX, enable);
    if (status != 0) {
        fprintf(stderr, "Failed to %s TX module: %s\n",
                enable ? "enable" : "disable",
                bladerf_strerror(status));
        return status;
    }

    return status;
}

static int init_if(struct bladerf *dev, struct app_params *p, bladerf_module m,
                   bool sync, bladerf_format fmt, int exp_status)
{
    int status;

    if (sync) {
        status = bladerf_sync_config(dev, m, fmt, p->num_buffers, p->buf_size,
                                     p->num_xfers, p->timeout_ms);
    } else {
        status = -1; /* TODO */
    }

    if (status != exp_status) {
        fprintf(stderr,
                "Did not receive expected status when configuring %s: %s\n",
                m == BLADERF_MODULE_RX ? "RX" : "TX", bladerf_strerror(status));
        status = -1;
    } else {
        status = 0;
    }

    return status;
}

static int deinit_ifs(struct bladerf *dev, const struct interfaces *ifs)
{
    return 0;
}

static void print_test_info(int n, const struct interfaces *i,
                            const struct test_case *t)
{
    printf("\nTest case %d\n", n);
    printf("-----------------------------------\n");
    printf(" RX i/f: %s\n", i->rx_sync ? "sync" : "async");
    printf(" RX fmt: %s\n", t->rx_fmt == BLADERF_FORMAT_SC16_Q11 ?
                            "SC16 Q11" : "SC16 Q11 + Metadata");
    printf(" TX i/f: %s\n", i->tx_sync ? "sync" : "async");
    printf(" TX fmt: %s\n", t->tx_fmt == BLADERF_FORMAT_SC16_Q11 ?
                            "SC16 Q11" : "SC16 Q11 + Metadata");
}

int test_fn_format_mismatch(struct bladerf *dev, struct app_params *p)
{
    int status;
    size_t i, j;

    for (i = 0; i < ARRAY_SIZE(ifs); i++) {

        for (j = 0; j < ARRAY_SIZE(tests); j++) {
            print_test_info(i * ARRAY_SIZE(tests) + j, &ifs[i], &tests[j]);

            /* Init RX first, then TX */
            printf("RX -> TX...");

            status = enable_modules(dev, false);
            if (status != 0) {
                goto fail;
            }

            status = init_if(dev, p, BLADERF_MODULE_RX, ifs[i].rx_sync,
                             tests[j].rx_fmt, 0);

            if (status != 0) {
                goto fail;
            }

            status = init_if(dev, p, BLADERF_MODULE_TX, ifs[i].tx_sync,
                             tests[j].tx_fmt, tests[j].exp_status);
            if (status != 0) {
                goto fail;
            }

            status = deinit_ifs(dev, &ifs[i]);
            if (status != 0) {
                goto fail;
            }

            printf("Pass.\n");

            /* Init TX first, then RX */
            printf("TX -> RX...");

            status = enable_modules(dev, false);
            if (status != 0) {
                goto fail;
            }

            status = init_if(dev, p, BLADERF_MODULE_TX, ifs[i].tx_sync,
                             tests[j].tx_fmt, 0);

            if (status != 0) {
                goto fail;
            }

            status = init_if(dev, p, BLADERF_MODULE_RX, ifs[i].rx_sync,
                             tests[j].rx_fmt, tests[j].exp_status);
            if (status != 0) {
                goto fail;
            }

            printf("Pass.\n");
        }
    }

fail:
    if (status != 0) {
        printf("Fail.\n");
    }

    printf("\n");
    return status;
}
