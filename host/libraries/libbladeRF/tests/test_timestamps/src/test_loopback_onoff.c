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

/* This test TX's some On-Off bursts and verifies the burst length and gaps
 * via loopback to the RX module */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <inttypes.h>
#include <pthread.h>
#include <libbladeRF.h>
#include "test_timestamps.h"
#include "minmax.h"

#define TX_MAGNITUDE    2000
#define RX_POWER_THRESH (1024 * 1024)

struct burst {
    uint64_t duration;
    uint64_t gap;
};

struct test {
    struct bladerf *dev;
    struct app_params *params;
    struct burst *bursts;
    size_t num_bursts;

    pthread_mutex_t lock;
    bool stop;
};


typedef enum {
    GET_SAMPLES,
    WAIT_FOR_BURST_START,
    WAIT_FOR_BURST_END,
} state;

static int16_t *init(struct test *t, bladerf_module m)
{
    int status;
    int16_t *samples;

    samples = malloc(2 * sizeof(samples[0]) * t->params->buf_size);
    if (samples == NULL) {
        perror("malloc");
        return NULL;
    }

    status = bladerf_enable_module(t->dev, m, true);
    if (status != 0) {
        fprintf(stderr, "Failed to enable RX module: %s\n",
                bladerf_strerror(status));

    }

    status = bladerf_sync_config(t->dev, m,
                                 BLADERF_FORMAT_SC16_Q11_META,
                                 t->params->num_buffers,
                                 t->params->buf_size,
                                 t->params->num_xfers,
                                 t->params->timeout_ms);
    if (status != 0) {
        fprintf(stderr, "Failed to configure RX stream: %s\n",
                bladerf_strerror(status));
        goto out;
    }

out:
    if (status != 0) {
        free(samples);
        return NULL;
    } else {
        return samples;
    }

}

void *rx_task(void *args)
{
    struct test *t = (struct test *) args;
    int16_t *samples;
    int status;
    size_t burst_num;
    struct bladerf_metadata meta;
    unsigned int idx;
    state curr_state, next_state;
    uint64_t burst_start, burst_end, burst_end_prev;
    bool stop;

    samples = init(t, BLADERF_MODULE_RX);
    if (samples == NULL) {
        return NULL;
    }

    samples = malloc(2 * sizeof(int16_t) * t->params->buf_size);
    if (samples == NULL) {
        perror("malloc");
        return NULL;
    }

    status = 0;
    burst_num = 0;

    memset(&meta, 0, sizeof(meta));
    meta.flags |= BLADERF_META_FLAG_RX_NOW;

    curr_state = GET_SAMPLES;
    next_state = WAIT_FOR_BURST_START;
    burst_start = burst_end = burst_end_prev = 0;
    stop = false;

    while (status == 0 && burst_num < t->num_bursts && !stop) {
        switch (curr_state) {
            case GET_SAMPLES:
                status = bladerf_sync_rx(t->dev, samples, t->params->buf_size,
                                         &meta, t->params->timeout_ms);

                if (status != 0) {
                    fprintf(stderr, "RX failed in burst %"PRIu64": %s\n",
                            (uint64_t)burst_num, bladerf_strerror(status));
                }

                idx = 0;
                curr_state = next_state;
                break;

            case WAIT_FOR_BURST_START:
                for (; idx < (2 * t->params->buf_size); idx += 2) {
                    const uint32_t sig_pow =
                        samples[idx] * samples[idx] +
                        samples[idx + 1] * samples[idx + 1];

                    if (sig_pow >= RX_POWER_THRESH) {
                        burst_start = meta.timestamp + (idx / 2);
                        burst_end_prev = burst_end;

                        printf("Burst detected @ %"PRIu64"\n", burst_start);

                        assert(burst_start > burst_end_prev);

                        if (burst_num != 0) {
                            const uint64_t gap = burst_start - burst_end_prev;
                            uint64_t delta;

                            if (gap > t->bursts[burst_num - 1].gap) {
                                delta = gap - t->bursts[burst_num].gap;
                            } else {
                                delta = t->bursts[burst_num].gap - gap;
                            }

                            if (delta > 1) {
                                status = BLADERF_ERR_UNEXPECTED;
                                fprintf(stderr, "Burst gap varied by %" PRIu64
                                        " samples.\n", delta);
                            } else {
                                printf("Burst gap = %"PRIu64", exp=%"PRIu64"\n",
                                       gap, t->bursts[burst_num].gap);
                                curr_state = WAIT_FOR_BURST_END;
                            }
                        }

                        break;
                    }
                }

                /* Need to fetch more samples */
                if (idx >= (2 * t->params->buf_size)) {
                    next_state = curr_state;
                    curr_state = GET_SAMPLES;
                }

                break;

            case WAIT_FOR_BURST_END:
                for (; idx < (2 * t->params->buf_size); idx += 2) {
                    const uint32_t sig_pow =
                        samples[idx] * samples[idx] +
                        samples[idx + 1] * samples[idx + 1];

                    if (sig_pow < RX_POWER_THRESH) {
                        uint64_t duration, delta;

                        burst_end = meta.timestamp + (idx / 2);
                        assert(burst_end > burst_start);

                        duration = burst_end - burst_start;
                        if (duration > t->bursts[burst_num].duration) {
                            delta  = duration - t->bursts[burst_num].duration;
                        } else {
                            delta  = t->bursts[burst_num].duration - duration;
                        }

                        if (delta > 1) {
                            status = BLADERF_ERR_UNEXPECTED;
                            fprintf(stderr, "Burst duration varied by %" PRIu64
                                    " samples.\n", delta);

                        } else {
                            curr_state = WAIT_FOR_BURST_START;
                        }
                    }
                }

                /* Need to fetch more samples */
                if (idx >= (2 * t->params->buf_size)) {
                    next_state = curr_state;
                    curr_state = GET_SAMPLES;
                }

                break;

            default:
                break;
        }

        if (status != 0) {
            fprintf(stderr, "RX failed for burst %"PRIu64": %s\n",
                    (uint64_t) burst_num, bladerf_strerror(status));
        }

        burst_num++;

        pthread_mutex_lock(&t->lock);
        stop = t->stop;
        pthread_mutex_unlock(&t->lock);
    }

    free(samples);
    bladerf_enable_module(t->dev, BLADERF_MODULE_RX, false);
    return NULL;
}

static void * tx_task(void *args)
{
    int status;
    int16_t *samples;
    size_t i;
    struct bladerf_metadata meta;
    unsigned int samples_left;
    struct test *t = (struct test *) args;

    samples = init(t, BLADERF_MODULE_TX);
    if (samples == NULL) {
        return NULL;
    }

    memset(&meta, 0, sizeof(meta));

    for (i = 0; i < (2 * t->params->buf_size); i += 2) {
        samples[i] = samples[i + 1] = TX_MAGNITUDE;
    }

    status = bladerf_get_timestamp(t->dev, BLADERF_MODULE_TX, &meta.timestamp);
    if (status != 0) {
        fprintf(stderr, "Failed to get current timestamp: %s\n",
                bladerf_strerror(status));
    }

    meta.timestamp += 200000;

    for (i = 0; i < t->num_bursts; i++) {
        meta.flags |= BLADERF_META_FLAG_TX_BURST_START;
        samples_left = t->bursts[i].duration;

        while (samples_left != 0 && status == 0) {
            unsigned int to_send = uint_min(t->params->buf_size, samples_left);

            if (to_send == samples_left) {
                meta.flags |= BLADERF_META_FLAG_TX_BURST_END;
            } else {
                meta.flags &= ~BLADERF_META_FLAG_TX_BURST_END;
            }

            status = bladerf_sync_tx(t->dev, samples, to_send, &meta,
                                     t->params->timeout_ms);

            if (status != 0) {
                fprintf(stderr, "Failed to TX @ burst %"PRIu64", with %u "
                        "samples left: %s\n",
                        i, samples_left, bladerf_strerror(status));

            }

            meta.flags &= ~BLADERF_META_FLAG_TX_BURST_START;
            samples_left -= to_send;
        }

        meta.timestamp += (t->bursts[i].duration + t->bursts[i].gap);
    }


    free(samples);
    bladerf_enable_module(t->dev, BLADERF_MODULE_TX, false);
    return NULL;
}

static inline void fill_bursts(struct test *t)
{
    size_t i;
    const unsigned int min_duration = 16;
    const unsigned int max_duration = 4 * t->params->buf_size;
    const unsigned int max_gap = 4 * t->params->buf_size;
    unsigned int min_gap;
    uint64_t randval_state;

    randval_init(&randval_state, 1);

    for (i = 0; i < t->num_bursts; i++) {
        randval_update(&randval_state);
        randval_state = randval_state % (max_duration - min_duration + 1);
        t->bursts[i].duration = (unsigned int) randval_state + min_duration;

        randval_update(&randval_state);
        min_gap = 16 + t->params->buf_size -
                            t->bursts[i].duration % t->params->buf_size;
        randval_state = randval_state % (max_gap - min_gap + 1);
        t->bursts[i].gap = (unsigned int) randval_state + min_gap;

        printf("Burst %-4"PRIu64": duration=%-8"PRIu64"  gap=%-8"PRIu64"\n",
               (uint64_t)i, t->bursts[i].duration, t->bursts[i].gap);
    }
}


int test_fn_loopback_onoff(struct bladerf *dev, struct app_params *p)
{
    int status = 0;
    struct test test;
    pthread_t tx_thread;
    bool tx_started;

    test.dev = dev;
    test.params = p;
    test.num_bursts = 2;
    test.stop = false;

    pthread_mutex_init(&test.lock, NULL);

    test.bursts = malloc(test.num_bursts * sizeof(test.bursts[0]));
    if (test.bursts == NULL) {
        perror("malloc");
        return -1;
    } else {
        fill_bursts(&test);
    }

    status = bladerf_set_loopback(dev, BLADERF_LB_NONE);
    if (status != 0) {
        fprintf(stderr, "Failed to set loopback mode: %s\n",
                bladerf_strerror(status));
    }

    status = pthread_create(&tx_thread, NULL, tx_task, &test);
    if (status != 0) {
        fprintf(stderr, "Failed to start TX thread: %s\n", strerror(status));
        tx_started = false;
        goto out;
    } else {
        tx_started = true;
    }

out:
    if (tx_started) {
        pthread_join(tx_thread, NULL);
    }

    free(test.bursts);

    bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
    bladerf_enable_module(dev, BLADERF_MODULE_TX, false);
    bladerf_set_loopback(dev, BLADERF_LB_NONE);

    return status;
}

