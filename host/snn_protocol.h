#ifndef SNN_PROTOCOL_H
#define SNN_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* =============================================================================
 * SNN Hardware Accelerator Parameters
 * Matches snn_top.v, cascaded_adder.v, master_fsm.v
 * ============================================================================= */

#define SNN_NUM_NEURONS         4096    /* N: Number of input neurons / weights */
#define SNN_WEIGHT_WIDTH_BITS   16      /* W: 16-bit signed weights */
#define SNN_WEIGHT_BYTES        8192    /* 4096 weights * 2 bytes = 8192 bytes */
#define SNN_TIMESTEPS           25      /* T_WINDOW: Rate coding observation window */

#define SNN_SPIKES_PER_STEP     4096    /* 4096 spike bits per timestep */
#define SNN_BYTES_PER_STEP      512     /* 4096 bits / 8 = 512 bytes per timestep */
#define SNN_TOTAL_SPIKE_BYTES   12800   /* 25 timesteps * 512 bytes = 12,800 bytes */

/* LIF Neuron & Rate Decoder Hardware Parameters */
#define SNN_LIF_THRESHOLD       1000    /* LIF firing threshold */
#define SNN_LIF_LEAK_SHIFT      3       /* Leak = membrane >> 3 */
#define SNN_LIF_RESET_VAL       0       /* Membrane reset potential */
#define SNN_SPIKE_THRESHOLD     (SNN_TIMESTEPS >> 1) /* 12: > 12 spikes => collision */

/* =============================================================================
 * Raw Ethernet Protocol Definitions
 * ============================================================================= */

#define DEFAULT_IFNAME          "eno1"
#define DEFAULT_FPGA_MAC_STR    "00:18:3E:04:C5:52"
#define SNN_ETH_TYPE            0x88B5  /* IEEE 802 Local Experimental EtherType */

/* Frame Command Types (Stored in Payload byte 0) */
#define SNN_ETH_CMD_WRITE_WEIGHTS   0xAA    /* Weight block transfer */
#define SNN_ETH_CMD_WRITE_SPIKES    0xBB    /* Spike timestep block transfer */
#define SNN_ETH_CMD_INFER_REQ       0xCC    /* Request inference result */
#define SNN_ETH_CMD_PING            0xDD    /* Ping / Status query */

/* Response Status / ACK Codes */
#define SNN_ETH_RESP_ACK            0x01    /* General ACK */
#define SNN_ETH_RESP_RESULT         0x02    /* Inference result frame */
#define SNN_ETH_RESP_PONG           0x03    /* Pong response */

/* Classification outputs */
#define SNN_CLASS_NO_COLLISION      0x00    /* 0 = No collision detected (safe) */
#define SNN_CLASS_COLLISION         0x01    /* 1 = Collision detected (obstacle) */

#endif /* SNN_PROTOCOL_H */
