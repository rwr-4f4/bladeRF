/* ... */

int status;

/* "User" buffer that we read samples into and do work on, and its associated
 * size, in units of samples. Recall that one sample = two int16_t values. */
int16_t *samples;
const size_t samples_len = 4096;

/* These items configure the underlying asynch stream used by the the sync
 * interface. The "buffer" here refers to those used internally by worker
 * threads, not the `samples` buffer above. */
const unsigned int num_buffers = 16;
const unsigned int buffer_size = 16384;
const unsigned int num_transfers = 8;
const unsigned int timeout_ms  = 3500;

 /* ... */

/* Allocate a buffer for 4096 IQ samples */
buffer = malloc(num_samples * 2 * sizeof(int16_t));
if (buffer == NULL) {
    perror("malloc");
    return BLADERF_ERR_MEM;
}

/* Configure the device's RX module for use with the sync interface.
 * SC16 Q11 samples *with* metadata are used. */
status = bladerf_sync_config(dev,
                             BLADERF_MODULE_RX,
                             BLADERF_FORMAT_SC16_Q11_META,
                             num_buffers,
                             buffer_size,
                             num_transfers,
                             timeout_ms);

if (status != 0) {
    fprintf(stderr, "Failed to configure RX sync interface: %s\n",
            bladerf_strerror(status));
    return status;
}

/* We must always enable the RX module *after* calling bladerf_sync_config(),
 * and *before* attempting to RX samples via bladerf_sync_rx(). */
status = bladerf_enable_module(dev, BLADERF_MODULE_RX, true);
if (status != 0) {
    fprintf(stderr, "Failed to enable RX module: %s\n",
            bladerf_strerror(status));
    return status;
}

/* Receive samples and do work on them */
while (status == 0 && !done) {
    status = bladerf_sync_rx(dev, samples, samples_len, NULL, 3500);
    if (status != 0) {
        fprintf(stderr, "Failed to RX samples: %s\n",
                bladerf_strerror(status));
    } else {
        if (

        done = do_work(samples, samples_len);
    }
}

/* Disable RX module, shutting down our underlying RX stream */
status = bladerf_enable_module(dev, BLADERF_MODULE_RX, false);
if (status != 0) {
    fprintf(stderr, "Failed to disable RX module: %s\n",
            bladerf_strerror(status));
}

/* Free up our resources */
free(samples);

/* ... */
