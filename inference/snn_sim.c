#include "snn_sim.h"
#include <stdio.h>
#include <string.h>

void snn_sim_config_default(snn_sim_config_t *cfg) {
    if (!cfg) return;
    cfg->threshold    = SNN_LIF_THRESHOLD;
    cfg->leak_shift   = SNN_LIF_LEAK_SHIFT;
    cfg->reset_val    = SNN_LIF_RESET_VAL;
    cfg->spike_thresh = SNN_SPIKE_THRESHOLD;
}

void snn_sim_run(const snn_sim_config_t *cfg,
                 const int16_t *weights,
                 const uint8_t *spike_stream,
                 snn_sim_result_t *result_out) {
    if (!weights || !spike_stream || !result_out) return;

    snn_sim_config_t local_cfg;
    if (cfg) {
        local_cfg = *cfg;
    } else {
        snn_sim_config_default(&local_cfg);
    }

    memset(result_out, 0, sizeof(snn_sim_result_t));

    int32_t membrane = local_cfg.reset_val;
    int total_spikes = 0;

    for (int t = 0; t < SNN_TIMESTEPS; t++) {
        const uint8_t *step_bytes = spike_stream + (t * SNN_BYTES_PER_STEP);
        int32_t mac_current = 0;

        /* Cascaded adder accumulation for 4096 neurons */
        for (int b = 0; b < SNN_BYTES_PER_STEP; b++) {
            uint8_t byte_val = step_bytes[b];
            for (int bit = 0; bit < 8; bit++) {
                int neuron_idx = b * 8 + bit;
                int spike = (byte_val >> (7 - bit)) & 1; /* MSB first */
                if (spike) {
                    mac_current += (int32_t)weights[neuron_idx];
                }
            }
        }

        /* LIF Neuron step: Leak -> Add Current -> Threshold Check */
        int32_t membrane_before = membrane;
        int32_t leak = membrane_before >> local_cfg.leak_shift; /* Arithmetic shift */
        int32_t next_membrane = membrane_before - leak + mac_current;
        int spike_fired = 0;

        if (next_membrane >= local_cfg.threshold) {
            membrane = local_cfg.reset_val;
            spike_fired = 1;
            total_spikes++;
        } else {
            membrane = next_membrane;
            spike_fired = 0;
        }

        result_out->steps[t].mac_current     = mac_current;
        result_out->steps[t].membrane_before = membrane_before;
        result_out->steps[t].leak_value      = leak;
        result_out->steps[t].membrane_after  = membrane;
        result_out->steps[t].spike           = spike_fired;
    }

    result_out->total_spikes   = total_spikes;
    result_out->final_membrane = membrane;
    result_out->collision      = (total_spikes > local_cfg.spike_thresh) ? 1 : 0;
}

void snn_sim_print_report(const snn_sim_result_t *result, bool verbose) {
    if (!result) return;

    printf("\n+-------------------------------------------------------------------+\n");
    printf("|              SNN ACCELERATOR INFERENCE SIMULATION                 |\n");
    printf("+-------+-------------+-------------+------------+------------------+\n");
    printf("| Step  | MAC Current | Leak Value  | LIF Spike  | Membrane Pot.    |\n");
    printf("+-------+-------------+-------------+------------+------------------+\n");

    for (int t = 0; t < SNN_TIMESTEPS; t++) {
        if (verbose || t < 5 || t >= SNN_TIMESTEPS - 5 || result->steps[t].spike) {
            printf("|  %2d   | %11d | %11d |     %d      | %16d |\n",
                   t,
                   result->steps[t].mac_current,
                   result->steps[t].leak_value,
                   result->steps[t].spike,
                   result->steps[t].membrane_after);
        } else if (t == 5 && !verbose) {
            printf("|  ..   |     ...     |     ...     |    ...     |        ...       |\n");
        }
    }

    printf("+-------+-------------+-------------+------------+------------------+\n");
    printf("| Total Spikes Fired: %2d / %2d (Threshold: >%d)                      |\n",
           result->total_spikes, SNN_TIMESTEPS, SNN_SPIKE_THRESHOLD);
    
    if (result->collision) {
        printf("| Result: [!] COLLISION DETECTED (Class 1)                          |\n");
    } else {
        printf("| Result: [OK] NO COLLISION - PATH CLEAR (Class 0)                  |\n");
    }
    printf("+-------------------------------------------------------------------+\n\n");
}
