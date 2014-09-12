#include <stdint.h>

#include "gmsk_burst.h"

static bool test_running = true ;
static struct bladerf *dev ;

static void *rx_task(void *args) {
    int status ;
    struct bladerf_metadata meta ;
    uint32_t power ;
    uint64_t time ;
    int16_t *start = NULL ;
    typedef {
        FIND_BURST,
        COUNT_BURST
    } rx_fsm_t ;

    rx_fsm_t rx_fsm = FIND_BURST ;

    meta.flags = BLADERF_META_FLAG_NOW ;
    while( test_running == true ) {
        /* Just read what we have there */
        status = bladerf_sync_rx(dev, (void *)samples, NUM_RX_SAMPLES, &meta, 2000) ;
        time = meta.timestamp ;
        samples_left = NUM_RX_SAMPLES ;
        if( status < 0 ) {
            fprintf( "liberror: %s\n", bladerf_strerror(status) ) ;
            continue ;
        }
        switch( rx_fsm ) {
            while( samples_left > 0 ) {
                case FIND_BURST:
                    /* Finding first sample with lots of power */
                    for( i = 0 ; i < samples_left ; i++ ) {
                        power = samples[2*i]*samples[2*i] + samples[2*i+1]*samples[2*i+1] ;
                        if( power > 100000 ) {
                            time += i ;
                            start = &samples[2*i] ;
                            samples_left -= i ;
                            count = 0 ;
                            rx_fsm = COUNT_BURST ;
                            fprintf( stderr, "Power found at timestamp %"PRIu64"\n", time ) ;
                            break ;
                        }
                    }
                    break ;

                case COUNT_BURST:
                    /* Counting all the samples with lots of power */
                    for( i = 0 ; i < samples_left ; i++, count++ ) {
                        power = start[2*i]*start[2*i] + samples[2*i+1]*samples[2*i+1] ;
                        if( power < 50000 ) {
                            samples_left -= i ;
                            rx_fsm = FIND_BURST ;
                            fprintf( stderr, "Burst length was %d\n", count ) ;
                            break ;
                        }
                    }
                    break ;

                default:
                    fprintf( stderr, "FSM error\n" ) ;
                    break ;
            }
        }
    }

    return NULL ;
}

static void *tx_task(void *args) {
    int status ;
    struct bladerf_metadata meta ;

    /* Burst is wholly contained inside of the single call */
    meta.flags = BLADERF_META_FLAG_BURST_START | BLADERF_META_FLAG_BURST_END ;

    /* Figure out a start time based on the current timestamp */
    status = bladerf_get_timestamp(dev, BLADERF_MODULE_TX, &meta.timestamp) ;
    if( status < 0 ) {
        fprintf( stderr, "Could not get initial timestamp\n" ) ;
        goto out ;
    }

    /* Add 1M samples to this */
    meta.timestamp += 1000000 ;
    while( test_running == true ) {
        status = bladerf_sync_tx(dev, (void *)gmsk_burst, gmsk_burst_num_samples, &meta, 2000) ;
        if( status < 0 ) {
            fprintf( stderr, "bladerf_sync_tx error: %s\n", bladerf_strerror(status) ) ;
        }
        /* Distance between beginning of each burst is 100k samples */
        meta.timestamp += 100000 - gmsk_burst_num_samples ;
    }
out:
    return NULL ;
}

