/*
 * Copyright (C) 2014 Nuand LLC
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 */
#include <errno.h>

/* Only switch on the verbose debug prints in this file when we *really* want
 * them. Otherwise, compile them out to avoid excessive log level checks
 * in our data path */
#include "log.h"
#ifndef ENABLE_LIBBLADERF_SYNC_LOG_VERBOSE
#undef log_verbose
#define log_verbose(...)
#endif

#include "bladerf_priv.h"
#include "async.h"
#include "sync.h"
#include "sync_worker.h"
#include "minmax.h"
#include "metadata.h"

static inline size_t samples2bytes(struct bladerf_sync *s, size_t n) {
    return s->stream_config.bytes_per_sample * n;
}

int sync_init(struct bladerf *dev,
              bladerf_module module,
              bladerf_format format,
              unsigned int num_buffers,
              unsigned int buffer_size,
              unsigned int num_transfers,
              unsigned int stream_timeout)

{
    struct bladerf_sync *sync;
    int status = 0;
    size_t i, bytes_per_sample;
    uint32_t gpio_val;

    if (num_transfers >= num_buffers) {
        return BLADERF_ERR_INVAL;
    }

    status = CONFIG_GPIO_READ(dev, &gpio_val);
    if (status != 0) {
        return status;
    }

    switch (format) {
        case BLADERF_FORMAT_SC16_Q11:
            bytes_per_sample = 4;

            /* Disable timestamps */
            gpio_val &= ~BLADERF_GPIO_TIMESTAMP;
            gpio_val &= ~BLADERF_GPIO_TIMESTAMP_DIV2;
            break;

        case BLADERF_FORMAT_SC16_Q11_META:
            bytes_per_sample = 4;

            /* Enable timestamps, with 1 count per sample */
            gpio_val |= (BLADERF_GPIO_TIMESTAMP | BLADERF_GPIO_TIMESTAMP_DIV2);
            break;

        default:
            log_debug("Invalid format value: %d\n", format);
            return BLADERF_ERR_INVAL;
    }

    status = CONFIG_GPIO_WRITE(dev, gpio_val);
    if (status != 0) {
        return status;
    }

    /* bladeRF GPIF DMA requirement */
    if ((bytes_per_sample * buffer_size) % 4096 != 0) {
        return BLADERF_ERR_INVAL;
    }

    /* Deallocate any existing sync handle for this module */
    switch (module) {
        case BLADERF_MODULE_TX:
        case BLADERF_MODULE_RX:
            sync_deinit(dev->sync[module]);
            sync = dev->sync[module] =
                (struct bladerf_sync *) calloc(1, sizeof(struct bladerf_sync));

            if (dev->sync[module] == NULL) {
                status = BLADERF_ERR_MEM;
            }
            break;


        default:
            log_debug("Invalid bladerf_module value encountered: %d", module);
            status = BLADERF_ERR_INVAL;
    }

    if (status != 0) {
        return status;
    }

    sync->dev = dev;
    sync->state = SYNC_STATE_CHECK_WORKER;

    sync->buf_mgmt.num_buffers = num_buffers;
    sync->buf_mgmt.resubmit_count = 0;

    sync->stream_config.module = module;
    sync->stream_config.format = format;
    sync->stream_config.samples_per_buffer = buffer_size;
    sync->stream_config.num_xfers = num_transfers;
    sync->stream_config.timeout_ms = stream_timeout;
    sync->stream_config.bytes_per_sample = bytes_per_sample;

    sync->meta.state = SYNC_META_STATE_HEADER;
    sync->meta.msg_per_buf = buffer_size / (dev->msg_size / bytes_per_sample);
    sync->meta.samples_per_msg =
        (dev->msg_size - METADATA_HEADER_SIZE) / bytes_per_sample;

    log_verbose("%s: Buffer size: %u\n",
                __FUNCTION__, buffer_size);

    log_verbose("%s: Msg per buffer: %u\n",
                __FUNCTION__, sync->meta.msg_per_buf);

    log_verbose("%s: Samples per msg: %u\n",
                __FUNCTION__, sync->meta.samples_per_msg);

    MUTEX_INIT(&sync->buf_mgmt.lock);
    pthread_cond_init(&sync->buf_mgmt.buf_ready, NULL);

    sync->buf_mgmt.status = (sync_buffer_status*) malloc(num_buffers * sizeof(sync_buffer_status));
    if (sync->buf_mgmt.status == NULL) {
        status = BLADERF_ERR_MEM;
    } else {
        switch (module) {
            case BLADERF_MODULE_RX:
                /* When starting up an RX stream, the first 'num_transfers'
                 * transfers will be submitted to the USB layer to grab data */
                sync->buf_mgmt.prod_i = num_transfers;
                sync->buf_mgmt.cons_i = 0;
                sync->buf_mgmt.partial_off = 0;

                for (i = 0; i < num_buffers; i++) {
                    if (i < num_transfers) {
                        sync->buf_mgmt.status[i] = SYNC_BUFFER_IN_FLIGHT;
                    } else {
                        sync->buf_mgmt.status[i] = SYNC_BUFFER_EMPTY;
                    }
                }

                sync->meta.msg_timestamp = 0;
                sync->meta.msg_flags = 0;

                break;

            case BLADERF_MODULE_TX:
                sync->buf_mgmt.prod_i = 0;
                sync->buf_mgmt.cons_i = 0;
                sync->buf_mgmt.partial_off = 0;

                for (i = 0; i < num_buffers; i++) {
                    sync->buf_mgmt.status[i] = SYNC_BUFFER_EMPTY;
                }

                sync->meta.in_burst = false;

                break;
        }

        status = sync_worker_init(sync);
    }

    if (status != 0) {
        sync_deinit(dev->sync[module]);
        dev->sync[module] = NULL;
    }

    return status;
}

void sync_deinit(struct bladerf_sync *sync)
{
    if (sync != NULL) {

        if (sync->stream_config.module == BLADERF_MODULE_TX) {
            async_submit_stream_buffer(sync->worker->stream,
                                       BLADERF_STREAM_SHUTDOWN, 0);
        }

        sync_worker_deinit(sync->worker, &sync->buf_mgmt.lock,
                           &sync->buf_mgmt.buf_ready);

         /* De-allocate our buffer management resources */
        free(sync->buf_mgmt.status);
        free(sync);
    }
}

static int wait_for_buffer(struct buffer_mgmt *b, unsigned int timeout_ms,
                           const char *dbg_name, unsigned int dbg_idx)
{
    int status;
    struct timespec timeout;

    if (timeout_ms == 0) {
        log_verbose("%s: Infinite wait for [%d] to fill.\n", dbg_name, dbg_idx);
        status = pthread_cond_wait(&b->buf_ready, &b->lock);
    } else {
        log_verbose("%s: Timed wait for [%d] to fill.\n", dbg_name, dbg_idx);
        status = populate_abs_timeout(&timeout, timeout_ms);
        if (status == 0) {
            status = pthread_cond_timedwait(&b->buf_ready, &b->lock, &timeout);
        }
    }

    if (status == ETIMEDOUT) {
        status = BLADERF_ERR_TIMEOUT;
    } else if (status != 0) {
        status = BLADERF_ERR_UNEXPECTED;
    }

    return status;
}

#ifndef SYNC_WORKER_START_TIMEOUT_MS
#   define SYNC_WORKER_START_TIMEOUT_MS 250
#endif

/* Returns # of samples left in a message (SC16Q11 mode only) */
static inline unsigned int left_in_msg(struct bladerf_sync *s)
{
    return s->meta.samples_per_msg - s->meta.curr_msg_off;
}

static inline void advance_rx_buffer(struct buffer_mgmt *b)
{
    log_verbose("%s: Marking buf[%u] empty.\n", __FUNCTION__, b->cons_i);

    b->status[b->cons_i] = SYNC_BUFFER_EMPTY;
    b->cons_i = (b->cons_i + 1) % b->num_buffers;
}

int sync_rx(struct bladerf *dev, void *samples, unsigned num_samples,
            struct bladerf_metadata *user_meta, unsigned int timeout_ms)
{
    struct bladerf_sync *s = dev->sync[BLADERF_MODULE_RX];
    struct buffer_mgmt *b;

    int status = 0;
    bool exit_early = false;
    bool copied_data = false;
    unsigned int samples_returned = 0;
    uint8_t *samples_dest = (uint8_t*)samples;
    uint8_t *buf_src = NULL;
    unsigned int samples_to_copy = 0;
    unsigned int samples_per_buffer = 0;
    uint64_t target_timestamp;

    if (s == NULL || samples == NULL) {
        log_debug("NULL pointer passed to %s\n", __FUNCTION__);
        return BLADERF_ERR_INVAL;
    } else if (s->stream_config.format == BLADERF_FORMAT_SC16_Q11_META) {
        if (user_meta == NULL) {
            log_debug("NULL metadata pointer passed to %s\n", __FUNCTION__);
            return BLADERF_ERR_INVAL;
        } else {
            target_timestamp = user_meta->timestamp;
        }
    }

    b = &s->buf_mgmt;
    samples_per_buffer = s->stream_config.samples_per_buffer;

    log_verbose("%s: Requests %u samples.\n", __FUNCTION__, num_samples);

    while (!exit_early && samples_returned < num_samples && status == 0) {

        switch (s->state) {
            case SYNC_STATE_CHECK_WORKER: {
                int stream_error;
                sync_worker_state worker_state =
                    sync_worker_get_state(s->worker, &stream_error);

                /* Propagate stream error back to the caller.
                 * They can call this function again to restart the stream and
                 * try again.
                 */
                if (stream_error != 0) {
                    status = stream_error;
                } else {
                    if (worker_state == SYNC_WORKER_STATE_IDLE) {
                        log_debug("%s: Worker is idle. Going to reset buf "
                                  "mgmt.\n", __FUNCTION__);
                        s->state = SYNC_STATE_RESET_BUF_MGMT;
                    } else if (worker_state == SYNC_WORKER_STATE_RUNNING) {
                        s->state = SYNC_STATE_WAIT_FOR_BUFFER;
                    } else {
                        status = BLADERF_ERR_UNEXPECTED;
                        log_debug("%s: Unexpected worker state=%d\n",
                                __FUNCTION__, worker_state);
                    }
                }

                break;
            }

            case SYNC_STATE_RESET_BUF_MGMT:
                MUTEX_LOCK(&b->lock);
                /* When the RX stream starts up, it will submit the first T
                 * transfers, so the consumer index must be reset to 0 */
                b->cons_i = 0;
                MUTEX_UNLOCK(&b->lock);
                log_debug("%s: Reset buf_mgmt consumer index\n", __FUNCTION__);
                s->state = SYNC_STATE_START_WORKER;
                break;


            case SYNC_STATE_START_WORKER:
                sync_worker_submit_request(s->worker, SYNC_WORKER_START);

                status = sync_worker_wait_for_state(
                                                s->worker,
                                                SYNC_WORKER_STATE_RUNNING,
                                                SYNC_WORKER_START_TIMEOUT_MS);

                if (status == 0) {
                    s->state = SYNC_STATE_WAIT_FOR_BUFFER;
                    log_debug("%s: Worker is now running.\n", __FUNCTION__);
                } else {
                    log_debug("%s: Failed to start worker, (%d)\n",
                              __FUNCTION__, status);
                }
                break;

            case SYNC_STATE_WAIT_FOR_BUFFER:
                MUTEX_LOCK(&b->lock);

                /* Check the buffer state, as the worker may have produced one
                 * since we last queried the status */
                if (b->status[b->cons_i] == SYNC_BUFFER_FULL) {
                    s->state = SYNC_STATE_BUFFER_READY;
                    log_verbose("%s: buffer %u is ready to consume\n",
                                __FUNCTION__, b->cons_i);
                } else {
                    status = wait_for_buffer(b, timeout_ms,
                                             __FUNCTION__, b->cons_i);

                    if (status == 0) {
                        if (b->status[b->cons_i] != SYNC_BUFFER_FULL) {
                            s->state = SYNC_STATE_CHECK_WORKER;
                        } else {
                            s->state = SYNC_STATE_BUFFER_READY;
                            log_verbose("%s: buffer %u is ready to consume\n",
                                        __FUNCTION__, b->cons_i);
                        }
                    }
                }

                MUTEX_UNLOCK(&b->lock);
                break;

            case SYNC_STATE_BUFFER_READY:
                MUTEX_LOCK(&b->lock);
                b->status[b->cons_i] = SYNC_BUFFER_PARTIAL;
                b->partial_off = 0;
                MUTEX_UNLOCK(&b->lock);

                switch (s->stream_config.format) {
                    case BLADERF_FORMAT_SC16_Q11:
                        s->state = SYNC_STATE_USING_BUFFER;
                        break;

                    case BLADERF_FORMAT_SC16_Q11_META:
                        s->state = SYNC_STATE_USING_BUFFER_META;
                        s->meta.curr_msg_off = 0;
                        s->meta.msg_num = 0;
                        break;

                    default:
                        assert(!"Invalid stream format");
                        status = BLADERF_ERR_UNEXPECTED;
                }
                break;

            case SYNC_STATE_USING_BUFFER: /* SC16Q11 buffers w/o metadata */
                MUTEX_LOCK(&b->lock);

                buf_src = (uint8_t*)b->buffers[b->cons_i];

                samples_to_copy = uint_min(num_samples - samples_returned,
                                           samples_per_buffer - b->partial_off);

                memcpy(samples_dest + samples2bytes(s, samples_returned),
                       buf_src + samples2bytes(s, b->partial_off),
                       samples2bytes(s, samples_to_copy));

                b->partial_off += samples_to_copy;
                samples_returned += samples_to_copy;

                log_verbose("%s: Provided %u samples to caller\n",
                            __FUNCTION__, samples_to_copy);

                /* We've finished consuming this buffer and can start looking
                 * for available samples in the next buffer */
                if (b->partial_off >= samples_per_buffer) {

                    /* Check for symptom of out-of-bounds accesses */
                    assert(b->partial_off == samples_per_buffer);

                    advance_rx_buffer(b);
                    s->state = SYNC_STATE_WAIT_FOR_BUFFER;
                }

                MUTEX_UNLOCK(&b->lock);
                break;


            case SYNC_STATE_USING_BUFFER_META: /* SC16Q11 buffers w/ metadata */
                MUTEX_LOCK(&b->lock);

                switch (s->meta.state) {
                    case SYNC_META_STATE_HEADER:

                        assert(s->meta.msg_num < s->meta.msg_per_buf);

                        buf_src = (uint8_t*)b->buffers[b->cons_i];

                        s->meta.curr_msg =
                            buf_src + dev->msg_size * s->meta.msg_num;

                        s->meta.msg_timestamp =
                            metadata_get_timestamp(s->meta.curr_msg);

                        s->meta.msg_flags =
                            metadata_get_flags(s->meta.curr_msg);

                        s->meta.curr_msg_off = 0;

                        /* We've encountered a discontinuity and need to return
                         * what we have so far, setting the status flags */
                        if (copied_data &&
                            s->meta.msg_timestamp != s->meta.curr_timestamp) {

                            user_meta->status |= BLADERF_META_STATUS_OVERRUN;
                            user_meta->actual_rx_samples = samples_returned;
                            exit_early = true;
                            log_debug("Sample discontinuity detected @ "
                                      "buffer %u, message %u: Expected t=%llu, "
                                      "got t=%llu\n",
                                      b->cons_i, s->meta.msg_num,
                                      (unsigned long long)s->meta.curr_timestamp,
                                      (unsigned long long)s->meta.msg_timestamp);

                        } else {
                            log_verbose("Got header for message %u: "
                                        "t_new=%u, t_old=%u\n",
                                        s->meta.msg_num,
                                        s->meta.msg_timestamp,
                                        s->meta.curr_timestamp);
                        }

                        s->meta.curr_timestamp = s->meta.msg_timestamp;
                        s->meta.state = SYNC_META_STATE_SAMPLES;
                        break;

                    case SYNC_META_STATE_SAMPLES:
                        if (!copied_data &&
                            (user_meta->flags & BLADERF_META_FLAG_RX_NOW) == 0 &&
                            target_timestamp < s->meta.curr_timestamp) {

                            log_debug("Current timestamp is %llu, "
                                      "target=%llu (user=%llu)\n",
                                      (unsigned long long)s->meta.curr_timestamp,
                                      (unsigned long long)target_timestamp,
                                      (unsigned long long)user_meta->timestamp);

                            status = BLADERF_ERR_TIME_PAST;
                        } else if ((user_meta->flags & BLADERF_META_FLAG_RX_NOW) ||
                                   target_timestamp == s->meta.curr_timestamp) {

                            /* Copy the request amount up to the end of a
                             * this message in the current buffer */
                            samples_to_copy =
                                uint_min(num_samples - samples_returned,
                                         left_in_msg(s));

                            memcpy(samples_dest + samples2bytes(s, samples_returned),
                                   s->meta.curr_msg +
                                        METADATA_HEADER_SIZE +
                                        samples2bytes(s, s->meta.curr_msg_off),
                                   samples2bytes(s, samples_to_copy));

                            samples_returned += samples_to_copy;
                            s->meta.curr_msg_off += samples_to_copy;

                            if (!copied_data &&
                                (user_meta->flags & BLADERF_META_FLAG_RX_NOW)) {

                                /* Provide the user with the timestamp at the
                                 * first returned sample when the
                                 * NOW flag has been provided */
                                user_meta->timestamp = s->meta.curr_timestamp;
                                log_verbose("Updated user meta timestamp with: "
                                            "%llu\n", (unsigned long long)
                                            user_meta->timestamp);
                            }

                            copied_data = true;
                            s->meta.curr_timestamp += samples_to_copy;

                            /* We've begun copying samples, so our target will
                             * just keep tracking the current timestamp. */
                            target_timestamp = s->meta.curr_timestamp;

                            log_verbose("After copying samples, t=%llu\n",
                                        (unsigned long long)s->meta.curr_timestamp);

                            if (left_in_msg(s) == 0) {
                                assert(s->meta.curr_msg_off == s->meta.samples_per_msg);

                                s->meta.state = SYNC_META_STATE_HEADER;
                                s->meta.msg_num++;

                                if (s->meta.msg_num >= s->meta.msg_per_buf) {
                                    assert(s->meta.msg_num == s->meta.msg_per_buf);
                                    advance_rx_buffer(b);
                                    s->meta.msg_num = 0;
                                    s->state = SYNC_STATE_WAIT_FOR_BUFFER;
                                }
                            }

                        } else {
                            const unsigned int time_delta =
                                target_timestamp - s->meta.curr_timestamp;

                            unsigned int left_in_buffer =
                                s->meta.samples_per_msg *
                                    (s->meta.msg_per_buf - s->meta.msg_num);

                            /* Account for current position in buffer */
                            left_in_buffer -= s->meta.curr_msg_off;

                            if (time_delta >= left_in_buffer) {
                                /* Discard the remainder of this buffer */
                                advance_rx_buffer(b);
                                s->state = SYNC_STATE_WAIT_FOR_BUFFER;
                                s->meta.state = SYNC_META_STATE_HEADER;

                                log_verbose("%s: Discarding rest of buffer.\n",
                                            __FUNCTION__);

                            } else if (time_delta <= left_in_msg(s)) {
                                /* Fast forward within the current message */
                                s->meta.curr_msg_off += time_delta;
                                s->meta.curr_timestamp += time_delta;

                                log_verbose("%s: Seeking within message (t=%llu)\n",
                                            __FUNCTION__,
                                            s->meta.curr_timestamp);
                            } else {
                                s->meta.state = SYNC_META_STATE_HEADER;

                                s->meta.msg_num +=
                                    time_delta / s->meta.samples_per_msg;

                                log_verbose("%s: Seeking to message %u.\n",
                                            __FUNCTION__, s->meta.msg_num);
                            }
                        }
                        break;

                    default:
                        assert(!"Invalid state");
                        status = BLADERF_ERR_UNEXPECTED;
                }

                MUTEX_UNLOCK(&b->lock);
                break;
        }
    }

    return status;
}

/* Assumes buffer lock is held */
static int advance_tx_buffer(struct bladerf_sync *s, struct buffer_mgmt *b)
{
    int status;

    log_verbose("%s: Marking buf[%u] full\n", __FUNCTION__, b->prod_i);
    b->status[b->prod_i] = SYNC_BUFFER_IN_FLIGHT;

    /* This call may block and it results in a per-stream lock being held, so
     * the buffer lock must be dropped.
     *
     * A callback may occur in the meantime, but this will not touch the status
     * for this this buffer, or the producer index.
     */
    MUTEX_UNLOCK(&b->lock);
    status = async_submit_stream_buffer(s->worker->stream,
                                        b->buffers[b->prod_i],
                                        s->stream_config.timeout_ms);
    MUTEX_LOCK(&b->lock);

    if (status == 0) {
        b->prod_i = (b->prod_i + 1) % b->num_buffers;

        /* Go handle the next buffer, if we have one available.  Otherwise,
         * check up on the worker's state and restart it if needed. */
        if (b->status[b->prod_i] == SYNC_BUFFER_EMPTY) {
            s->state = SYNC_STATE_BUFFER_READY;
        } else {
            s->state = SYNC_STATE_CHECK_WORKER;
        }
    } else {
        log_debug("%s: Failed to advance buffer: %s\n",
                  __FUNCTION__, bladerf_strerror(status));
    }

    return status;
}

int sync_tx(struct bladerf *dev, void *samples, unsigned int num_samples,
             struct bladerf_metadata *user_meta, unsigned int timeout_ms)
{
    struct bladerf_sync *s = dev->sync[BLADERF_MODULE_TX];
    struct buffer_mgmt *b = NULL;

    int status = 0;
    unsigned int samples_written = 0;
    unsigned int samples_to_copy = 0;
    unsigned int samples_per_buffer = 0;
    uint8_t *samples_src = (uint8_t*)samples;
    uint8_t *buf_dest = NULL;
    bool flush = false;

    if (s == NULL || samples == NULL) {
        return BLADERF_ERR_INVAL;
    }

    if (s->stream_config.format == BLADERF_FORMAT_SC16_Q11_META) {
        if (user_meta == NULL) {
            log_debug("NULL metadata pointer passed to %s\n", __FUNCTION__);
            return BLADERF_ERR_INVAL;
        }

        log_verbose("%s: called for %u samples.\n", __FUNCTION__, num_samples);

        if (user_meta->flags & BLADERF_META_FLAG_TX_BURST_START) {
            if (s->meta.in_burst) {
                log_debug("%s: BURST_START provided while already in a burst.\n",
                          __FUNCTION__);
                return BLADERF_ERR_INVAL;
            } else if (user_meta->timestamp < s->meta.curr_timestamp) {
                log_debug("Provided timestamp=%llu is in past: "
                          "current=%llu\n",
                          (unsigned long long)user_meta->timestamp,
                          (unsigned long long)s->meta.curr_timestamp);

                return BLADERF_ERR_TIME_PAST;
            } else {
                s->meta.in_burst = true;
                s->meta.curr_timestamp = user_meta->timestamp;
                log_verbose("%s: Starting burst @ %llu\n", __FUNCTION__,
                            (unsigned long long)s->meta.curr_timestamp);
            }
        }

        if (user_meta->flags & BLADERF_META_FLAG_TX_BURST_END) {
            if (s->meta.in_burst) {
                flush = true;
            } else {
                log_debug("%s: BURST_END provided while not in a burst.\n",
                          __FUNCTION__);
                return BLADERF_ERR_INVAL;
            }
        }
    }

    b = &s->buf_mgmt;
    samples_per_buffer = s->stream_config.samples_per_buffer;

    while (status == 0 && ((samples_written < num_samples) || flush) ) {

        switch (s->state) {
            case SYNC_STATE_CHECK_WORKER: {
                int stream_error;
                sync_worker_state worker_state =
                    sync_worker_get_state(s->worker, &stream_error);

                if (stream_error != 0) {
                    status = stream_error;
                } else {
                    if (worker_state == SYNC_WORKER_STATE_IDLE) {
                        /* No need to reset any buffer managment for TX since
                         * the TX stream does not submit an initial set of
                         * buffers.  Therefore the RESET_BUF_MGMT state is
                         * skipped here. */
                        s->state = SYNC_STATE_START_WORKER;
                    }
                }
                break;
            }

            case SYNC_STATE_RESET_BUF_MGMT:
                assert(!"Bug");
                break;

            case SYNC_STATE_START_WORKER:
                sync_worker_submit_request(s->worker, SYNC_WORKER_START);

                status = sync_worker_wait_for_state(
                        s->worker,
                        SYNC_WORKER_STATE_RUNNING,
                        SYNC_WORKER_START_TIMEOUT_MS);

                if (status == 0) {
                    s->state = SYNC_STATE_WAIT_FOR_BUFFER;
                    log_debug("%s: Worker is now running.\n", __FUNCTION__);
                }
                break;

            case SYNC_STATE_WAIT_FOR_BUFFER:
                MUTEX_LOCK(&b->lock);

                /* Check the buffer state, as the worker may have consumed one
                 * since we last queried the status */
                if (b->status[b->prod_i] == SYNC_BUFFER_EMPTY) {
                    s->state = SYNC_STATE_BUFFER_READY;
                } else {
                    status = wait_for_buffer(b, timeout_ms,
                                             __FUNCTION__, b->prod_i);
                }

                MUTEX_UNLOCK(&b->lock);
                break;

            case SYNC_STATE_BUFFER_READY:
                MUTEX_LOCK(&b->lock);
                b->status[b->prod_i] = SYNC_BUFFER_PARTIAL;
                b->partial_off = 0;
                MUTEX_UNLOCK(&b->lock);

                switch (s->stream_config.format) {
                    case BLADERF_FORMAT_SC16_Q11:
                        s->state = SYNC_STATE_USING_BUFFER;
                        break;

                    case BLADERF_FORMAT_SC16_Q11_META:
                        s->state = SYNC_STATE_USING_BUFFER_META;
                        s->meta.curr_msg_off = 0;
                        s->meta.msg_num = 0;
                        break;

                    default:
                        assert(!"Invalid stream format");
                        status = BLADERF_ERR_UNEXPECTED;
                }
                break;


            case SYNC_STATE_USING_BUFFER:
                MUTEX_LOCK(&b->lock);

                buf_dest = (uint8_t*)b->buffers[b->prod_i];
                samples_to_copy = uint_min(num_samples - samples_written,
                                           samples_per_buffer - b->partial_off);

                memcpy(buf_dest + samples2bytes(s, b->partial_off),
                        samples_src + samples2bytes(s, samples_written),
                        samples2bytes(s, samples_to_copy));

                b->partial_off += samples_to_copy;
                samples_written += samples_to_copy;

                log_verbose("%s: Buffered %u samples from caller\n",
                            __FUNCTION__, samples_to_copy);

                if (b->partial_off >= samples_per_buffer) {
                    /* Check for symptom of out-of-bounds accesses */
                    assert(b->partial_off == samples_per_buffer);

                    /* Submit buffer and advance to the next one */
                    status = advance_tx_buffer(s, b);
                }

                MUTEX_UNLOCK(&b->lock);
                break;

            case SYNC_STATE_USING_BUFFER_META: /* SC16Q11 buffers w/ metadata */
                MUTEX_LOCK(&b->lock);

                switch (s->meta.state) {

                    case SYNC_META_STATE_HEADER:
                        buf_dest = (uint8_t*)b->buffers[b->prod_i];

                        s->meta.curr_msg =
                            buf_dest + dev->msg_size * s->meta.msg_num;

                        log_verbose("%s: Set curr_msg to: %p (buf @ %p)\n",
                                     __FUNCTION__, s->meta.curr_msg, buf_dest);

                        s->meta.curr_msg_off = 0;

                        metadata_set(s->meta.curr_msg,
                                     s->meta.curr_timestamp, 0);

                        s->meta.state = SYNC_META_STATE_SAMPLES;

                        log_verbose("%s: Filled in header (t=%llu)\n",
                                    __FUNCTION__,
                                    (unsigned long long)s->meta.curr_timestamp);
                        break;

                    case SYNC_META_STATE_SAMPLES:
                        samples_to_copy =
                            uint_min(num_samples - samples_written,
                                     left_in_msg(s));

                        if (samples_to_copy != 0) {
                            /* We have user data to copy into the current
                             * message within the buffer */
                            memcpy(s->meta.curr_msg + METADATA_HEADER_SIZE +
                                        samples2bytes(s, s->meta.curr_msg_off),
                                   samples + samples2bytes(s, samples_written),
                                   samples2bytes(s, samples_to_copy));

                            s->meta.curr_msg_off += samples_to_copy;
                            s->meta.curr_timestamp += samples_to_copy;
                            samples_written += samples_to_copy;

                            log_verbose("%s: Copied %u samples. "
                                        "Currrent message offset is now: %u\n",
                                        __FUNCTION__, samples_to_copy,
                                        s->meta.curr_msg_off);

                        }

                        if (left_in_msg(s) != 0 && flush) {
                            /* We're ending this buffer early and need to
                             * flush the remaining samples by setting all
                             * samples in the messages to (0 + 0j) */
                            const unsigned int to_zero = left_in_msg(s);

                            const size_t off =
                                METADATA_HEADER_SIZE +
                                samples2bytes(s, s->meta.curr_msg_off);

                            /* If we're here, we should have already copied
                             * all requested data to the buffer */
                            assert(num_samples == samples_written);

                            memset(s->meta.curr_msg + off, 0,
                                   samples2bytes(s, to_zero));

                            log_verbose("%s: Flushed %u samples @ %u (0x%08x)\n",
                                        __FUNCTION__,
                                        to_zero, s->meta.curr_msg_off, off);

                            s->meta.curr_msg_off += to_zero;
                            s->meta.curr_timestamp += to_zero;

                        }

                        if (left_in_msg(s) == 0) {
                            s->meta.msg_num++;
                            s->meta.state = SYNC_META_STATE_HEADER;

                            log_verbose("%s: Advancing to next message (%u)\n",
                                        __FUNCTION__, s->meta.msg_num);
                        }

                        if (s->meta.msg_num >= s->meta.msg_per_buf) {
                            assert(s->meta.msg_num == s->meta.msg_per_buf);

                            /* Submit buffer of samples for transmission */
                            status = advance_tx_buffer(s, b);

                            s->meta.msg_num = 0;
                            s->state = SYNC_STATE_WAIT_FOR_BUFFER;

                            /* We want to clear the flush flag if we've written
                             * all of our data, but keep it set if we have more
                             * data and need wrap around to another buffer */
                            flush = flush && (samples_written != num_samples);
                        }

                        break;

                    default:
                        assert(!"Invalid state");
                        status = BLADERF_ERR_UNEXPECTED;
                }

                MUTEX_UNLOCK(&b->lock);
                break;
        }
    }

    if (s->stream_config.format == BLADERF_FORMAT_SC16_Q11_META &&
        (user_meta->flags & BLADERF_META_FLAG_TX_BURST_END)) {
        s->meta.in_burst = false;
    }

    return status;
}

unsigned int sync_buf2idx(struct buffer_mgmt *b, void *addr)
{
    unsigned int i;

    for (i = 0; i < b->num_buffers; i++) {
        if (b->buffers[i] == addr) {
            return i;
        }
    }

    assert(!"Bug: Buffer not found.");

    /* Assertions are intended to always remain on. If someone turned them
     * off, do the best we can...complain loudly and clobber a buffer */
    log_critical("Bug: Buffer not found.");
    return 0;
}

