#include "eth_comm.h"
#include "snn_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <errno.h>

#define ETH_BUFFER_SIZE 1518

int eth_init(eth_handle_t *handle, const char *ifname, const char *fpga_mac_str) {
    if (!handle) return -1;
    memset(handle, 0, sizeof(eth_handle_t));
    strncpy(handle->ifname, ifname ? ifname : DEFAULT_IFNAME, sizeof(handle->ifname) - 1);

    // 1. Create raw packet socket
    handle->sock_fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (handle->sock_fd < 0) {
        perror("socket(AF_PACKET) failed (run as root / sudo)");
        return -1;
    }

    // Default receive timeout: 500ms
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    setsockopt(handle->sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    // 2. Get interface index and host MAC address
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, handle->ifname, IFNAMSIZ - 1);

    if (ioctl(handle->sock_fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX) failed");
        close(handle->sock_fd);
        handle->sock_fd = -1;
        return -2;
    }
    handle->ifindex = ifr.ifr_ifindex;

    if (ioctl(handle->sock_fd, SIOCGIFHWADDR, &ifr) < 0) {
        perror("ioctl(SIOCGIFHWADDR) failed");
        close(handle->sock_fd);
        handle->sock_fd = -1;
        return -3;
    }
    memcpy(handle->host_mac, ifr.ifr_hwaddr.sa_data, 6);

    // 3. Parse FPGA MAC string
    const char *mac_str = fpga_mac_str ? fpga_mac_str : DEFAULT_FPGA_MAC_STR;
    sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
           &handle->fpga_mac[0], &handle->fpga_mac[1], &handle->fpga_mac[2],
           &handle->fpga_mac[3], &handle->fpga_mac[4], &handle->fpga_mac[5]);

    return 0;
}

void eth_close(eth_handle_t *handle) {
    if (handle && handle->sock_fd >= 0) {
        close(handle->sock_fd);
        handle->sock_fd = -1;
    }
}

static int eth_send_raw(eth_handle_t *handle, const uint8_t *payload, size_t payload_len) {
    if (!handle || handle->sock_fd < 0 || !payload) return -1;

    uint8_t tx_frame[ETH_BUFFER_SIZE];
    memset(tx_frame, 0, sizeof(tx_frame));

    // Header: [Dest MAC (6)] [Src MAC (6)] [EtherType (2)]
    memcpy(tx_frame, handle->fpga_mac, 6);
    memcpy(tx_frame + 6, handle->host_mac, 6);
    tx_frame[12] = (SNN_ETH_TYPE >> 8) & 0xFF;
    tx_frame[13] = SNN_ETH_TYPE & 0xFF;

    // Payload
    memcpy(tx_frame + 14, payload, payload_len);

    size_t total_len = 14 + payload_len;
    if (total_len < 60) total_len = 60; // Ethernet minimum 60 bytes before FCS

    struct sockaddr_ll sll;
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = handle->ifindex;
    sll.sll_halen = ETH_ALEN;
    memcpy(sll.sll_addr, handle->fpga_mac, 6);

    ssize_t sent = sendto(handle->sock_fd, tx_frame, total_len, 0, (struct sockaddr*)&sll, sizeof(sll));
    return (sent == (ssize_t)total_len) ? 0 : -1;
}

static int eth_recv_packet(eth_handle_t *handle, uint8_t *rx_payload_out, size_t *payload_len_out, int timeout_ms) {
    if (!handle || handle->sock_fd < 0) return -1;

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(handle->sock_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    uint8_t rx_frame[ETH_BUFFER_SIZE];
    while (1) {
        ssize_t bytes_rx = recvfrom(handle->sock_fd, rx_frame, sizeof(rx_frame), 0, NULL, NULL);
        if (bytes_rx < 0) {
            return -2; // Timeout or error
        }

        // Filter: Destination MAC == Host MAC, Source MAC == FPGA MAC, EtherType == 0x88B5
        if (bytes_rx >= 14 &&
            memcmp(rx_frame, handle->host_mac, 6) == 0 &&
            memcmp(rx_frame + 6, handle->fpga_mac, 6) == 0) {
            
            uint16_t ethertype = (rx_frame[12] << 8) | rx_frame[13];
            if (ethertype == SNN_ETH_TYPE) {
                size_t p_len = (size_t)(bytes_rx - 14);
                if (rx_payload_out && payload_len_out) {
                    memcpy(rx_payload_out, rx_frame + 14, p_len);
                    *payload_len_out = p_len;
                }
                return 0; // Success
            }
        }
    }
}

int eth_ping(eth_handle_t *handle, int *weights_loaded, int *fifo_empty, int timeout_ms) {
    uint8_t tx_payload[2] = {SNN_ETH_CMD_PING, 0x00};
    if (eth_send_raw(handle, tx_payload, sizeof(tx_payload)) != 0) {
        return -1;
    }

    uint8_t rx_payload[64];
    size_t rx_len = 0;
    if (eth_recv_packet(handle, rx_payload, &rx_len, timeout_ms) != 0) {
        return -2; // Timeout
    }

    if (rx_payload[0] == SNN_ETH_RESP_PONG) {
        if (weights_loaded) *weights_loaded = (int)rx_payload[1];
        if (fifo_empty)    *fifo_empty    = (int)rx_payload[2];
        return 0;
    }
    return -3;
}

int eth_send_weights(eth_handle_t *handle, const int16_t *weights, size_t num_weights,
                     void (*progress_cb)(size_t sent, size_t total)) {
    if (!handle || !weights || num_weights != SNN_NUM_NEURONS) {
        return -1;
    }

    // Format 4096 16-bit weights as 8192 Big-Endian bytes
    uint8_t weight_bytes[SNN_WEIGHT_BYTES];
    for (size_t i = 0; i < num_weights; i++) {
        weight_bytes[2 * i]     = (uint8_t)((weights[i] >> 8) & 0xFF);
        weight_bytes[2 * i + 1] = (uint8_t)(weights[i] & 0xFF);
    }

    // Send in 8 blocks (1024 bytes per frame)
    const size_t block_size = 1024;
    for (int blk = 0; blk < 8; blk++) {
        uint8_t frame_payload[2 + block_size];
        frame_payload[0] = SNN_ETH_CMD_WRITE_WEIGHTS;
        frame_payload[1] = (uint8_t)blk;
        memcpy(frame_payload + 2, weight_bytes + (blk * block_size), block_size);

        if (eth_send_raw(handle, frame_payload, sizeof(frame_payload)) != 0) {
            return -2;
        }

        // Await ACK response
        uint8_t rx_payload[64];
        size_t rx_len = 0;
        if (eth_recv_packet(handle, rx_payload, &rx_len, 2000) != 0) {
            return -3; // Timeout on block ACK
        }

        if (rx_payload[0] != SNN_ETH_RESP_ACK || rx_payload[1] != (uint8_t)blk) {
            return -4; // Invalid ACK
        }

        if (progress_cb) {
            progress_cb((blk + 1) * block_size, SNN_WEIGHT_BYTES);
        }
        usleep(100);
    }

    return 0;
}

int eth_send_spikes(eth_handle_t *handle, const uint8_t *spike_stream, size_t total_bytes,
                    int *out_result, void (*progress_cb)(int step, int total_steps)) {
    if (!handle || !spike_stream || total_bytes != SNN_TOTAL_SPIKE_BYTES) {
        return -1;
    }

    // Stream 25 timesteps, each 512 bytes
    for (int t = 0; t < SNN_TIMESTEPS; t++) {
        uint8_t frame_payload[2 + SNN_BYTES_PER_STEP];
        frame_payload[0] = SNN_ETH_CMD_WRITE_SPIKES;
        frame_payload[1] = (uint8_t)t;
        memcpy(frame_payload + 2, spike_stream + (t * SNN_BYTES_PER_STEP), SNN_BYTES_PER_STEP);

        if (eth_send_raw(handle, frame_payload, sizeof(frame_payload)) != 0) {
            return -2;
        }

        // Await timestep ACK
        uint8_t rx_payload[64];
        size_t rx_len = 0;
        if (eth_recv_packet(handle, rx_payload, &rx_len, 2000) != 0) {
            return -3; // Timestep ACK timeout
        }

        if (rx_payload[0] != SNN_ETH_RESP_ACK || rx_payload[1] != (uint8_t)t) {
            return -4;
        }

        if (progress_cb) {
            progress_cb(t + 1, SNN_TIMESTEPS);
        }
    }

    // Wait for the final classification result packet emitted by FPGA
    uint8_t res_payload[64];
    size_t res_len = 0;
    if (eth_recv_packet(handle, res_payload, &res_len, 500) == 0) {
        if (res_payload[0] == SNN_ETH_RESP_RESULT) {
            if (out_result) *out_result = (int)res_payload[1];
        } else {
            if (out_result) *out_result = -1;
        }
    } else {
        if (out_result) *out_result = -1;
    }

    return 0;
}

int eth_request_result(eth_handle_t *handle, int *out_result, int timeout_ms) {
    uint8_t tx_payload[2] = {SNN_ETH_CMD_INFER_REQ, 0x00};
    if (eth_send_raw(handle, tx_payload, sizeof(tx_payload)) != 0) {
        return -1;
    }

    uint8_t rx_payload[64];
    size_t rx_len = 0;
    if (eth_recv_packet(handle, rx_payload, &rx_len, timeout_ms) != 0) {
        return -2;
    }

    if (rx_payload[0] == SNN_ETH_RESP_RESULT) {
        if (out_result) *out_result = (int)rx_payload[1];
        return 0;
    }
    return -3;
}
