#ifndef FILE_IO_H
#define FILE_IO_H

#include "snn_protocol.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WEIGHT_PATTERN_RANDOM = 0,
    WEIGHT_PATTERN_OBSTACLE_DETECTOR = 1,
    WEIGHT_PATTERN_UNIFORM_POSITIVE = 2,
    WEIGHT_PATTERN_UNIFORM_NEGATIVE = 3
} weight_pattern_t;

typedef enum {
    SCENARIO_CLEAR_PATH = 0,        /* No obstacle (low/no spikes -> class 0) */
    SCENARIO_OBSTACLE_COLLISION = 1,/* Center obstacle (dense spikes -> class 1) */
    SCENARIO_SIDE_OBSTACLE = 2,     /* Side obstacle -> class 1 */
    SCENARIO_ALL_ONES = 3,          /* Max activity (0xFF) */
    SCENARIO_ALL_ZEROS = 4          /* Min activity (0x00) */
} spike_scenario_t;

/**
 * Load weights from a file (auto-detects binary 16-bit or text/hex format).
 * 
 * @param filepath    Path to file
 * @param weights_out Destination buffer for 4096 signed 16-bit weights
 * @param count       Expected number of weights (SNN_NUM_NEURONS)
 * @return 0 on success, negative error code on failure
 */
int file_load_weights(const char *filepath, int16_t *weights_out, size_t count);

/**
 * Save weights to a binary file (8192 bytes, Big-Endian).
 */
int file_save_weights_bin(const char *filepath, const int16_t *weights, size_t count);

/**
 * Save weights to a human-readable text file (decimal format).
 */
int file_save_weights_txt(const char *filepath, const int16_t *weights, size_t count);

/**
 * Generate synthetic weight patterns for testing.
 */
void file_generate_weights(int16_t *weights_out, size_t count, weight_pattern_t pattern);

/**
 * Load spike stream or image from file.
 * Auto-detects 12,800-byte full spike stream, 512-byte single frame, 4096-byte pixel array,
 * PBM/PGM 64x64 image files, or text/CSV formats.
 * Automatically rate-codes or replicates single frames to 25 timesteps (12,800 bytes).
 * 
 * @param filepath          Path to image or spike file
 * @param spike_stream_out  Destination buffer (12,800 bytes)
 * @return 0 on success, negative error code on failure
 */
int file_load_spikes_or_image(const char *filepath, uint8_t *spike_stream_out);

/**
 * Save spike stream to a binary file (12,800 bytes).
 */
int file_save_spikes_bin(const char *filepath, const uint8_t *spike_stream, size_t total_bytes);

/**
 * Generate synthetic 64x64 image/spike scenario over 25 timesteps.
 */
void file_generate_scenario(uint8_t *spike_stream_out, spike_scenario_t scenario);

/**
 * Render an ASCII representation of a 64x64 frame to console.
 * 
 * @param frame_512B 512 bytes representing 4096 bits (64x64)
 */
void file_render_ascii_image(const uint8_t *frame_512B);

/**
 * Check if path is a directory.
 */
bool file_is_directory(const char *path);

/**
 * Scan a directory for supported image/spike files (.pgm, .ppm, .pbm, .bin, .dat, .txt).
 * Returns array of allocated strings sorted alphabetically.
 * 
 * @param dirpath       Directory path to scan
 * @param filepaths_out Destination pointer for array of string paths
 * @param count_out     Destination pointer for number of found files
 * @return 0 on success, negative error code on failure
 */
int file_scan_directory(const char *dirpath, char ***filepaths_out, int *count_out);

/**
 * Free the memory allocated by file_scan_directory.
 */
void file_free_scanned_list(char **filepaths, int count);

#ifdef __cplusplus
}
#endif

#endif /* FILE_IO_H */
