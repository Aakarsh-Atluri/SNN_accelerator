#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Open and configure a serial port for 8N1 communication.
 * 
 * @param port_name Path to device (e.g., "/dev/ttyUSB0")
 * @param baudrate  Baud rate (e.g., 115200)
 * @return File descriptor on success, -1 on failure
 */
int uart_open(const char *port_name, int baudrate);

/**
 * Close an open serial port.
 * 
 * @param fd File descriptor returned by uart_open
 */
void uart_close(int fd);

/**
 * Flush input and output buffers.
 * 
 * @param fd File descriptor
 * @return 0 on success, -1 on failure
 */
int uart_flush(int fd);

/**
 * Write a buffer of bytes to the serial port.
 * 
 * @param fd   File descriptor
 * @param data Pointer to data buffer
 * @param len  Number of bytes to write
 * @return Number of bytes written, or -1 on error
 */
ssize_t uart_write_all(int fd, const uint8_t *data, size_t len);

/**
 * Read bytes from the serial port with a timeout.
 * 
 * @param fd         File descriptor
 * @param buffer     Destination buffer
 * @param len        Number of bytes to read
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes read, or -1 on error/timeout
 */
ssize_t uart_read_timeout(int fd, uint8_t *buffer, size_t len, int timeout_ms);

/**
 * Read a single byte with timeout.
 * 
 * @param fd         File descriptor
 * @param byte_out   Pointer to store read byte
 * @param timeout_ms Timeout in milliseconds
 * @return 0 on success, -1 on timeout/error
 */
int uart_read_byte(int fd, uint8_t *byte_out, int timeout_ms);

/**
 * Automatically scan for available serial ports (e.g. /dev/ttyUSB*, /dev/ttyACM*).
 * 
 * @param found_ports  Array of strings to fill with port names
 * @param max_ports    Maximum ports to detect
 * @return Number of detected ports
 */
int uart_detect_ports(char found_ports[][64], int max_ports);

/**
 * Send weights to FPGA and verify write completion.
 * 
 * Protocol:
 * 1. Send header 0xAA.
 * 2. Send 8192 bytes (4096 x 16-bit big-endian).
 * 3. Send query 0xBB.
 * 4. Await 0x01 ACK from FPGA.
 * 
 * @param fd           Serial port file descriptor
 * @param weights      Array of 4096 signed 16-bit weights
 * @param num_weights  Number of weights (must be SNN_NUM_NEURONS)
 * @param progress_cb  Optional callback function(current_bytes, total_bytes)
 * @return 0 on success (verified), negative error code on failure
 */
int uart_send_weights(int fd, const int16_t *weights, size_t num_weights,
                      void (*progress_cb)(size_t sent, size_t total));

/**
 * Stream image spikes across 25 timesteps to FPGA and retrieve results.
 * 
 * Protocol:
 * 1. Send 0xBB -> Receive 0x01 ACK.
 * 2. For each of 25 timesteps:
 *    - Send 512 bytes (4096 spike bits).
 *    - Wait for 0x01 ACK.
 * 3. Read inference result if available.
 * 
 * @param fd           Serial port file descriptor
 * @param spike_stream Array of 12,800 bytes (25 * 512 bytes)
 * @param total_bytes  Total bytes (must be SNN_TOTAL_SPIKE_BYTES)
 * @param out_result   Pointer to store hardware result byte if received (optional)
 * @param progress_cb  Optional callback function(timestep, total_timesteps)
 * @return 0 on success, negative error code on failure
 */
int uart_send_spikes(int fd, const uint8_t *spike_stream, size_t total_bytes,
                     int *out_result, void (*progress_cb)(int step, int total_steps));

#ifdef __cplusplus
}
#endif

#endif /* UART_H */
