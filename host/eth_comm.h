#ifndef ETH_COMM_H
#define ETH_COMM_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

typedef struct {
    int sock_fd;
    int ifindex;
    unsigned char host_mac[6];
    unsigned char fpga_mac[6];
    char ifname[32];
} eth_handle_t;

/**
 * Initialize raw Ethernet socket binding to given network interface.
 * @param handle Structure to populate
 * @param ifname Network interface name (e.g. "eno1")
 * @param fpga_mac_str MAC string for FPGA (e.g. "00:18:3E:04:C5:52")
 * @return 0 on success, negative error code on failure
 */
int eth_init(eth_handle_t *handle, const char *ifname, const char *fpga_mac_str);

/**
 * Close raw Ethernet socket.
 */
void eth_close(eth_handle_t *handle);

/**
 * Send Ping command (0xDD) to FPGA and check for Pong reply (0x03).
 * @return 0 on success, negative error code on timeout/failure
 */
int eth_ping(eth_handle_t *handle, int *weights_loaded, int *fifo_empty, int timeout_ms);

/**
 * Send 8192 bytes of weights in 8 blocks (1024 bytes each) with per-block ACK verification.
 */
int eth_send_weights(eth_handle_t *handle, const int16_t *weights, size_t num_weights,
                     void (*progress_cb)(size_t sent, size_t total));

/**
 * Stream 25 timesteps of spikes (512 bytes each) with per-timestep ACK verification.
 * Also retrieves final classification result from FPGA.
 */
int eth_send_spikes(eth_handle_t *handle, const uint8_t *spike_stream, size_t total_bytes,
                    int *out_result, void (*progress_cb)(int step, int total_steps));

/**
 * Request inference result explicitly via 0xCC command.
 */
int eth_request_result(eth_handle_t *handle, int *out_result, int timeout_ms);

#endif /* ETH_COMM_H */
