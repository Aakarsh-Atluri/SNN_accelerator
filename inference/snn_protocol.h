#ifndef SNN_PROTOCOL_H
#define SNN_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* =============================================================================
 * SNN Hardware Accelerator Parameters
 * Matches snn_top.v, fifo_uart_controller.v, master_fsm.v
 * ============================================================================= */

#define SNN_NUM_NEURONS         4096    /* N: Number of input neurons / weights */
#define SNN_WEIGHT_WIDTH_BITS   16      /* W: 16-bit signed weights */
#define SNN_WEIGHT_BYTES        8192    /* 4096 weights * 2 bytes = 8192 bytes */
#define SNN_TIMESTEPS           25      /* T_WINDOW: Rate coding observation window */

#define SNN_SPIKES_PER_STEP     4096    /* 4096 spike bits per timestep */
#define SNN_BYTES_PER_STEP      512     /* 4096 bits / 8 = 512 bytes per timestep */
#define SNN_TOTAL_SPIKE_BYTES   12800   /* 25 timesteps * 512 bytes = 12,800 bytes */

#define SNN_DEFAULT_BAUD        115200  /* UART Baud rate */
#define SNN_DEFAULT_PORT        "/dev/ttyUSB0"

/* LIF Neuron & Rate Decoder Hardware Parameters */
#define SNN_LIF_THRESHOLD       1000    /* LIF firing threshold */
#define SNN_LIF_LEAK_SHIFT      3       /* Leak = membrane >> 3 */
#define SNN_LIF_RESET_VAL       0       /* Membrane reset potential */
#define SNN_SPIKE_THRESHOLD     (SNN_TIMESTEPS >> 1) /* 12: > 12 spikes => collision */

/* =============================================================================
 * UART Protocol Magic Control Bytes
 * ============================================================================= */

#define SNN_CMD_WRITE_WEIGHTS   0xAA    /* Header sent to begin receiving 8192 weight bytes */
#define SNN_CMD_START_SPIKES    0xBB    /* Command to begin spike stream reception / ping */
#define SNN_RESP_ACK            0x01    /* ACK returned by FPGA (on 0xBB and after every 512B) */

/* Classification outputs */
#define SNN_CLASS_NO_COLLISION  0x00    /* 0 = No collision detected (safe) */
#define SNN_CLASS_COLLISION     0x01    /* 1 = Collision detected (obstacle) */

#endif /* SNN_PROTOCOL_H */
