#ifndef SNN_SIM_H
#define SNN_SIM_H

#include "snn_protocol.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t threshold;   /* Default: 1000 */
    int32_t leak_shift;  /* Default: 3 */
    int32_t reset_val;   /* Default: 0 */
    int32_t spike_thresh;/* Default: 12 */
} snn_sim_config_t;

typedef struct {
    int32_t mac_current;    /* Sum of active weights */
    int32_t membrane_before;/* Membrane before leak + current */
    int32_t leak_value;     /* Membrane >> leak_shift */
    int32_t membrane_after; /* Membrane after update / reset */
    int     spike;          /* 1 if fired, 0 if silent */
} snn_step_log_t;

typedef struct {
    snn_step_log_t steps[SNN_TIMESTEPS];
    int            total_spikes;    /* Number of timesteps where neuron spiked */
    int            collision;       /* 1 = Collision, 0 = No collision */
    int32_t        final_membrane;
} snn_sim_result_t;

/**
 * Initialize simulation configuration with default hardware parameters.
 */
void snn_sim_config_default(snn_sim_config_t *cfg);

/**
 * Run bit-accurate simulation of the SNN hardware accelerator.
 * 
 * @param cfg          Simulation configuration (or NULL for default)
 * @param weights      Array of 4096 16-bit signed weights
 * @param spike_stream Array of 12,800 packed spike bytes (25 timesteps * 512 bytes)
 * @param result_out   Destination structure for simulation result
 */
void snn_sim_run(const snn_sim_config_t *cfg,
                 const int16_t *weights,
                 const uint8_t *spike_stream,
                 snn_sim_result_t *result_out);

/**
 * Print detailed analysis report of the simulation run.
 */
void snn_sim_print_report(const snn_sim_result_t *result, bool verbose);

#ifdef __cplusplus
}
#endif

#endif /* SNN_SIM_H */
