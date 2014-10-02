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
#include <stdio.h>
#include <libbladeRF.h>

struct bladerf *example_init(const char *devstr)
{
    int status;
    struct bladerf *dev;
    const unsigned int samplerate = 2000000;
    const unsigned int bw = BLADERF_BANDWIDTH_MIN;

    const unsigned int freq_rx = 910000000;
    const bladerf_lna_gain lna_gain = BLADERF_LNA_GAIN_MAX;
    const int rxvga1 = 20;
    const int rxvga2 = BLADERF_RXVGA2_GAIN_MIN;;
    const unsigned int freq_tx = 920000000;

    const int txvga1 = -20;
    const int txvga2 = BLADERF_TXVGA2_GAIN_MIN;

    printf("Opening and initializing device...\n\n");

    status = bladerf_open(&dev, devstr);
    if (status != 0) {
        fprintf(stderr, "Failed to open device: %s\n",
                bladerf_strerror(status));
        goto out;
    }

    status = bladerf_set_frequency(dev, BLADERF_MODULE_RX, freq_rx);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX frequency: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("RX frequency: %u Hz\n", freq_rx);
    }

    status = bladerf_set_sample_rate(dev, BLADERF_MODULE_RX, samplerate, NULL);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX sample rate: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("RX samplerate: %u sps\n", samplerate);
    }

    status = bladerf_set_bandwidth(dev, BLADERF_MODULE_RX, bw, NULL);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX bandwidth: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("RX bandwidth: %u Hz\n", bw);
    }

    status = bladerf_set_lna_gain(dev, lna_gain);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX LNA gain: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("RX LNA Gain: Max\n");
    }

    status = bladerf_set_rxvga1(dev, rxvga1);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX VGA1 gain: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("RX VGA1 gain: %d\n", rxvga1);
    }

    status = bladerf_set_rxvga2(dev, rxvga2);
    if (status != 0) {
        fprintf(stderr, "Failed to set RX VGA2 gain: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("RX VGA2 gain: %d\n\n", rxvga2);
    }

    status = bladerf_set_frequency(dev, BLADERF_MODULE_TX, freq_tx);
    if (status != 0) {
        fprintf(stderr, "Faield to set TX frequency: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("TX frequency: %u Hz\n", freq_tx);
    }

    status = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX, samplerate, NULL);
    if (status != 0) {
        fprintf(stderr, "Failed to set TX sample rate: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("TX samplerate: %u sps\n", samplerate);
    }

    status = bladerf_set_bandwidth(dev, BLADERF_MODULE_TX, bw, NULL);
    if (status != 0) {
        fprintf(stderr, "Failed to set TX bandwidth: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("TX bandwidth: %u\n", bw);
    }

    status = bladerf_set_txvga1(dev, txvga1);
    if (status != 0) {
        fprintf(stderr, "Failed to set TX VGA1 gain: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("TX VGA1 gain: %d\n", txvga1);
    }

    status = bladerf_set_txvga2(dev, txvga2);
    if (status != 0) {
        fprintf(stderr, "Failed to set TX VGA2 gain: %s\n",
                bladerf_strerror(status));
        goto out;
    } else {
        printf("TX VGA2 gain: %d\n\n", txvga2);
    }

out:
    if (status != 0) {
        bladerf_close(dev);
        return NULL;
    } else {
        return dev;
    }
}
