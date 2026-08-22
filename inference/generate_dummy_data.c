#include "snn_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Helper to pack a 64x64 binary pixel array into 512 bytes (MSB first) */
static void pack_64x64_to_512B(const uint8_t *pixels, uint8_t out_512B[512]) {
    memset(out_512B, 0, 512);
    for (int r = 0; r < 64; r++) {
        for (int c = 0; c < 64; c++) {
            int i = r * 64 + c;
            if (pixels[i]) {
                int byte_idx = i / 8;
                int bit_idx = 7 - (i % 8); /* MSB first matching spike_unpacker.v */
                out_512B[byte_idx] |= (1 << bit_idx);
            }
        }
    }
}

int main(void) {
    printf("=================================================================\n");
    printf("  Generating Dummy Weights & Synthesized Images for SNN Hardware  \n");
    printf("=================================================================\n\n");

    /* -----------------------------------------------------------------
     * 1. CREATE DUMMY WEIGHTS
     * Pattern:
     *   - Center 32x32 obstacle zone (rows 16..47, cols 16..47): weight = +25
     *   - Outer periphery (surrounding background): weight = -2
     * ----------------------------------------------------------------- */
    int16_t weights[4096];
    uint8_t weights_be[8192];

    for (int r = 0; r < 64; r++) {
        for (int c = 0; c < 64; c++) {
            int idx = r * 64 + c;
            if (r >= 16 && r < 48 && c >= 16 && c < 48) {
                weights[idx] = 25; /* Positive excitation */
            } else {
                weights[idx] = -2; /* Small inhibition */
            }
            /* Big-Endian format for FPGA UART controller */
            weights_be[2 * idx]     = (uint8_t)((weights[idx] >> 8) & 0xFF);
            weights_be[2 * idx + 1] = (uint8_t)(weights[idx] & 0xFF);
        }
    }

    /* Save binary weights (8192 bytes) */
    FILE *fw_bin = fopen("dummy_weights.bin", "wb");
    if (fw_bin) {
        fwrite(weights_be, 1, 8192, fw_bin);
        fclose(fw_bin);
        printf("[+] Generated 'dummy_weights.bin' (8,192 bytes, Big-Endian raw binary)\n");
    }

    /* Save text weights (4096 lines) */
    FILE *fw_txt = fopen("dummy_weights.txt", "w");
    if (fw_txt) {
        for (int i = 0; i < 4096; i++) {
            fprintf(fw_txt, "%d\n", (int)weights[i]);
        }
        fclose(fw_txt);
        printf("[+] Generated 'dummy_weights.txt' (4,096 decimal lines)\n");
    }

    /* -----------------------------------------------------------------
     * 2. CREATE SYNTHESIZED COLLISION IMAGE
     * Pattern:
     *   - Solid 24x24 box obstacle centered at rows 20..43, cols 20..43 (576 pixels)
     *   - Current per timestep = 576 * 25 = 14,400 >> 1000 (LIF Threshold)
     *   - Fires 25/25 spikes -> SNN Classification: 1 (COLLISION)
     * ----------------------------------------------------------------- */
    uint8_t collision_pixels[64 * 64];
    memset(collision_pixels, 0, sizeof(collision_pixels));

    for (int r = 20; r < 44; r++) {
        for (int c = 20; c < 44; c++) {
            collision_pixels[r * 64 + c] = 1;
        }
    }

    uint8_t collision_512B[512];
    pack_64x64_to_512B(collision_pixels, collision_512B);

    /* 12,800 bytes stream (25 timesteps) */
    uint8_t collision_stream[12800];
    for (int t = 0; t < 25; t++) {
        memcpy(collision_stream + (t * 512), collision_512B, 512);
    }

    FILE *fc_bin = fopen("dummy_collision_image.bin", "wb");
    if (fc_bin) {
        fwrite(collision_stream, 1, 12800, fc_bin);
        fclose(fc_bin);
        printf("[+] Generated 'dummy_collision_image.bin' (12,800 bytes, 25 timesteps)\n");
    }

    /* Save as standard PBM (Portable BitMap) image for viewing */
    FILE *fc_pbm = fopen("dummy_collision_image.pbm", "w");
    if (fc_pbm) {
        fprintf(fc_pbm, "P1\n# Synthesized Collision Obstacle (64x64)\n64 64\n");
        for (int r = 0; r < 64; r++) {
            for (int c = 0; c < 64; c++) {
                fprintf(fc_pbm, "%d ", collision_pixels[r * 64 + c]);
            }
            fprintf(fc_pbm, "\n");
        }
        fclose(fc_pbm);
        printf("[+] Generated 'dummy_collision_image.pbm' (64x64 standard visual image)\n");
    }

    /* -----------------------------------------------------------------
     * 3. CREATE SYNTHESIZED NO-COLLISION / CLEAR PATH IMAGE
     * Pattern:
     *   - Clear path with only 4 sparse background pixels
     *   - Current per timestep = 4 * 25 = 100 < 1000 (LIF Threshold)
     *   - Fires 0/25 spikes -> SNN Classification: 0 (NO COLLISION)
     * ----------------------------------------------------------------- */
    uint8_t clear_pixels[64 * 64];
    memset(clear_pixels, 0, sizeof(clear_pixels));
    clear_pixels[5 * 64 + 10]  = 1;
    clear_pixels[12 * 64 + 55] = 1;
    clear_pixels[50 * 64 + 8]  = 1;
    clear_pixels[58 * 64 + 45] = 1;

    uint8_t clear_512B[512];
    pack_64x64_to_512B(clear_pixels, clear_512B);

    uint8_t clear_stream[12800];
    for (int t = 0; t < 25; t++) {
        memcpy(clear_stream + (t * 512), clear_512B, 512);
    }

    FILE *fcl_bin = fopen("dummy_no_collision_image.bin", "wb");
    if (fcl_bin) {
        fwrite(clear_stream, 1, 12800, fcl_bin);
        fclose(fcl_bin);
        printf("[+] Generated 'dummy_no_collision_image.bin' (12,800 bytes, 25 timesteps)\n");
    }

    FILE *fcl_pbm = fopen("dummy_no_collision_image.pbm", "w");
    if (fcl_pbm) {
        fprintf(fcl_pbm, "P1\n# Synthesized Clear Road Image (64x64)\n64 64\n");
        for (int r = 0; r < 64; r++) {
            for (int c = 0; c < 64; c++) {
                fprintf(fcl_pbm, "%d ", clear_pixels[r * 64 + c]);
            }
            fprintf(fcl_pbm, "\n");
        }
        fclose(fcl_pbm);
        printf("[+] Generated 'dummy_no_collision_image.pbm' (64x64 standard visual image)\n");
    }

    printf("\n[SUCCESS] All dummy files generated successfully in 'inference/'.\n");
    return 0;
}
