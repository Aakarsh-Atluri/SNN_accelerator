#include "file_io.h"
#include "snn_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>

static void pack_64x64_to_512B(const uint8_t *pixels_4096, uint8_t *frame_512B) {
    memset(frame_512B, 0, SNN_BYTES_PER_STEP);
    for (int i = 0; i < SNN_SPIKES_PER_STEP; i++) {
        if (pixels_4096[i]) {
            int byte_idx = i / 8;
            int bit_idx  = 7 - (i % 8); /* MSB first */
            frame_512B[byte_idx] |= (1 << bit_idx);
        }
    }
}

int file_load_weights(const char *filepath, int16_t *weights_out, size_t count) {
    if (!filepath || !weights_out || count != SNN_NUM_NEURONS) {
        return -1;
    }

    struct stat st;
    if (stat(filepath, &st) != 0) {
        return -2;
    }

    FILE *f = fopen(filepath, "rb");
    if (!f) return -3;

    /* 1. Exact binary file of 8192 bytes (4096 x 16-bit words) */
    if (st.st_size == (off_t)SNN_WEIGHT_BYTES) {
        uint8_t raw_bytes[SNN_WEIGHT_BYTES];
        if (fread(raw_bytes, 1, SNN_WEIGHT_BYTES, f) != SNN_WEIGHT_BYTES) {
            fclose(f);
            return -4;
        }
        fclose(f);

        /* Interpret as Big-Endian signed 16-bit words */
        for (size_t i = 0; i < count; i++) {
            uint16_t u = ((uint16_t)raw_bytes[2 * i] << 8) | (uint16_t)raw_bytes[2 * i + 1];
            weights_out[i] = (int16_t)u;
        }
        return 0;
    }

    /* 2. Text / CSV / HEX file */
    fseek(f, 0, SEEK_SET);
    size_t loaded = 0;
    char token[128];
    while (loaded < count && fscanf(f, "%127s", token) == 1) {
        /* Strip trailing commas/semicolons */
        size_t len = strlen(token);
        while (len > 0 && (token[len - 1] == ',' || token[len - 1] == ';')) {
            token[--len] = '\0';
        }
        if (len == 0) continue;

        long val;
        if (token[0] == '0' && (token[1] == 'x' || token[1] == 'X')) {
            val = strtol(token, NULL, 16);
            if (val > 32767) val -= 65536; /* Handle 16-bit signed hex */
        } else {
            val = strtol(token, NULL, 10);
        }

        weights_out[loaded++] = (int16_t)val;
    }
    fclose(f);

    if (loaded == count) {
        return 0;
    }

    return -5; /* Incomplete text file */
}

int file_save_weights_bin(const char *filepath, const int16_t *weights, size_t count) {
    if (!filepath || !weights || count != SNN_NUM_NEURONS) return -1;

    FILE *f = fopen(filepath, "wb");
    if (!f) return -2;

    uint8_t raw[SNN_WEIGHT_BYTES];
    for (size_t i = 0; i < count; i++) {
        raw[2 * i]     = (uint8_t)((weights[i] >> 8) & 0xFF);
        raw[2 * i + 1] = (uint8_t)(weights[i] & 0xFF);
    }

    size_t written = fwrite(raw, 1, SNN_WEIGHT_BYTES, f);
    fclose(f);
    return (written == SNN_WEIGHT_BYTES) ? 0 : -3;
}

int file_save_weights_txt(const char *filepath, const int16_t *weights, size_t count) {
    if (!filepath || !weights || count != SNN_NUM_NEURONS) return -1;

    FILE *f = fopen(filepath, "w");
    if (!f) return -2;

    for (size_t i = 0; i < count; i++) {
        fprintf(f, "%d\n", (int)weights[i]);
    }
    fclose(f);
    return 0;
}

void file_generate_weights(int16_t *weights_out, size_t count, weight_pattern_t pattern) {
    if (!weights_out || count != SNN_NUM_NEURONS) return;

    srand(12345);

    switch (pattern) {
        case WEIGHT_PATTERN_OBSTACLE_DETECTOR:
            /* Obstacle detector: Center 32x32 region in 64x64 field has strong positive weights */
            for (int r = 0; r < 64; r++) {
                for (int c = 0; c < 64; c++) {
                    int idx = r * 64 + c;
                    if (r >= 16 && r < 48 && c >= 16 && c < 48) {
                        /* Center region: positive weights (20 to 45) */
                        weights_out[idx] = (int16_t)(20 + (rand() % 26));
                    } else {
                        /* Periphery: slightly inhibitory / negative (-5 to 0) */
                        weights_out[idx] = (int16_t)(-5 + (rand() % 6));
                    }
                }
            }
            break;

        case WEIGHT_PATTERN_UNIFORM_POSITIVE:
            for (size_t i = 0; i < count; i++) {
                weights_out[i] = 15;
            }
            break;

        case WEIGHT_PATTERN_UNIFORM_NEGATIVE:
            for (size_t i = 0; i < count; i++) {
                weights_out[i] = -10;
            }
            break;

        case WEIGHT_PATTERN_RANDOM:
        default:
            for (size_t i = 0; i < count; i++) {
                weights_out[i] = (int16_t)(-15 + (rand() % 35));
            }
            break;
    }
}

/**
 * Universal Netpbm PGM / PBM / PPM parser with arbitrary resolution support.
 * Automatically downsamples / resizes to 64x64 grayscale buffer using bilinear interpolation.
 */
static int parse_pbm_pgm_any_size(FILE *f, uint8_t *pixels_64x64_out, int *orig_w, int *orig_h) {
    char magic[3] = {0};
    if (fscanf(f, "%2s", magic) != 1) return -1;

    /* Skip comments */
    int c;
    while ((c = fgetc(f)) != EOF) {
        if (isspace(c)) continue;
        if (c == '#') {
            while ((c = fgetc(f)) != EOF && c != '\n');
        } else {
            ungetc(c, f);
            break;
        }
    }

    int width = 0, height = 0, maxval = 255;
    if (fscanf(f, "%d %d", &width, &height) != 2 || width <= 0 || height <= 0) return -2;

    if (strcmp(magic, "P2") == 0 || strcmp(magic, "P5") == 0 ||
        strcmp(magic, "P3") == 0 || strcmp(magic, "P6") == 0) {
        /* Skip comments before maxval */
        while ((c = fgetc(f)) != EOF) {
            if (isspace(c)) continue;
            if (c == '#') {
                while ((c = fgetc(f)) != EOF && c != '\n');
            } else {
                ungetc(c, f);
                break;
            }
        }
        if (fscanf(f, "%d", &maxval) != 1 || maxval <= 0) return -3;
    }
    fgetc(f); /* Consume single whitespace after header */

    if (orig_w) *orig_w = width;
    if (orig_h) *orig_h = height;

    size_t num_pixels = (size_t)width * (size_t)height;
    uint8_t *src_gray = (uint8_t *)malloc(num_pixels);
    if (!src_gray) return -10;

    if (strcmp(magic, "P5") == 0) {
        /* PGM Binary (raw grayscale, e.g. Zurich dataset 324x244 P5) */
        if (maxval < 256) {
            if (fread(src_gray, 1, num_pixels, f) != num_pixels) {
                free(src_gray);
                return -8;
            }
        } else {
            /* 16-bit PGM -> read 2 bytes per pixel, take MSB */
            for (size_t i = 0; i < num_pixels; i++) {
                int hi = fgetc(f);
                int lo = fgetc(f);
                if (hi == EOF || lo == EOF) { free(src_gray); return -8; }
                src_gray[i] = (uint8_t)hi;
            }
        }
    } else if (strcmp(magic, "P2") == 0) {
        /* PGM ASCII */
        for (size_t i = 0; i < num_pixels; i++) {
            int val = 0;
            if (fscanf(f, "%d", &val) != 1) { free(src_gray); return -7; }
            src_gray[i] = (uint8_t)((val * 255) / maxval);
        }
    } else if (strcmp(magic, "P4") == 0) {
        /* PBM Binary (packed bits: (width + 7)/8 bytes per row) */
        int row_bytes = (width + 7) / 8;
        uint8_t *row_buf = (uint8_t *)malloc(row_bytes);
        if (!row_buf) { free(src_gray); return -10; }
        for (int r = 0; r < height; r++) {
            if (fread(row_buf, 1, row_bytes, f) != (size_t)row_bytes) {
                free(row_buf); free(src_gray); return -6;
            }
            for (int col = 0; col < width; col++) {
                int byte_idx = col / 8;
                int bit_idx = 7 - (col % 8);
                int bit = (row_buf[byte_idx] >> bit_idx) & 1;
                src_gray[r * width + col] = bit ? 255 : 0;
            }
        }
        free(row_buf);
    } else if (strcmp(magic, "P1") == 0) {
        /* PBM ASCII (0 or 1) */
        for (size_t i = 0; i < num_pixels; i++) {
            int bit = 0;
            if (fscanf(f, "%d", &bit) != 1) { free(src_gray); return -5; }
            src_gray[i] = bit ? 255 : 0;
        }
    } else if (strcmp(magic, "P6") == 0) {
        /* PPM Binary (RGB) */
        for (size_t i = 0; i < num_pixels; i++) {
            int r = fgetc(f), g = fgetc(f), b = fgetc(f);
            if (r == EOF || g == EOF || b == EOF) { free(src_gray); return -8; }
            src_gray[i] = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
        }
    } else if (strcmp(magic, "P3") == 0) {
        /* PPM ASCII (RGB) */
        for (size_t i = 0; i < num_pixels; i++) {
            int r = 0, g = 0, b = 0;
            if (fscanf(f, "%d %d %d", &r, &g, &b) != 3) { free(src_gray); return -7; }
            src_gray[i] = (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
        }
    } else {
        free(src_gray);
        return -9; /* Unsupported format */
    }

    /* High quality bilinear downsampling/resizing to 64x64 */
    if (width == 64 && height == 64) {
        for (int i = 0; i < 4096; i++) {
            pixels_64x64_out[i] = src_gray[i];
        }
    } else {
        for (int r = 0; r < 64; r++) {
            float gy = ((float)r + 0.5f) * ((float)height / 64.0f) - 0.5f;
            int y0 = (int)gy;
            if (y0 < 0) y0 = 0;
            if (y0 >= height - 1) y0 = height - 1;
            int y1 = (y0 < height - 1) ? y0 + 1 : y0;
            float dy = gy - (float)y0;
            if (dy < 0.0f) dy = 0.0f;
            if (dy > 1.0f) dy = 1.0f;

            for (int col = 0; col < 64; col++) {
                float gx = ((float)col + 0.5f) * ((float)width / 64.0f) - 0.5f;
                int x0 = (int)gx;
                if (x0 < 0) x0 = 0;
                if (x0 >= width - 1) x0 = width - 1;
                int x1 = (x0 < width - 1) ? x0 + 1 : x0;
                float dx = gx - (float)x0;
                if (dx < 0.0f) dx = 0.0f;
                if (dx > 1.0f) dx = 1.0f;

                float p00 = (float)src_gray[y0 * width + x0];
                float p10 = (float)src_gray[y0 * width + x1];
                float p01 = (float)src_gray[y1 * width + x0];
                float p11 = (float)src_gray[y1 * width + x1];

                float interp = (1.0f - dx) * (1.0f - dy) * p00 +
                               dx * (1.0f - dy) * p10 +
                               (1.0f - dx) * dy * p01 +
                               dx * dy * p11;

                if (interp < 0.0f) interp = 0.0f;
                if (interp > 255.0f) interp = 255.0f;

                pixels_64x64_out[r * 64 + col] = (uint8_t)(interp + 0.5f);
            }
        }
    }

    free(src_gray);
    return 0;
}

int file_load_spikes_or_image(const char *filepath, uint8_t *spike_stream_out) {
    if (!filepath || !spike_stream_out) return -1;

    struct stat st;
    if (stat(filepath, &st) != 0) return -2;

    FILE *f = fopen(filepath, "rb");
    if (!f) return -3;

    /* Case 1: Exact 12,800 bytes full spike stream binary */
    if (st.st_size == (off_t)SNN_TOTAL_SPIKE_BYTES) {
        if (fread(spike_stream_out, 1, SNN_TOTAL_SPIKE_BYTES, f) != SNN_TOTAL_SPIKE_BYTES) {
            fclose(f);
            return -4;
        }
        fclose(f);
        return 0;
    }

    /* Case 2: Exact 512 bytes single binary frame -> Replicate over 25 timesteps */
    if (st.st_size == (off_t)SNN_BYTES_PER_STEP) {
        uint8_t single_frame[SNN_BYTES_PER_STEP];
        if (fread(single_frame, 1, SNN_BYTES_PER_STEP, f) != SNN_BYTES_PER_STEP) {
            fclose(f);
            return -5;
        }
        fclose(f);
        for (int t = 0; t < SNN_TIMESTEPS; t++) {
            memcpy(spike_stream_out + (t * SNN_BYTES_PER_STEP), single_frame, SNN_BYTES_PER_STEP);
        }
        return 0;
    }

    /* Case 3: Try Universal Netpbm PGM / PBM / PPM parser with Auto-Resize */
    uint8_t gray_64x64[4096];
    int orig_w = 0, orig_h = 0;
    if (parse_pbm_pgm_any_size(f, gray_64x64, &orig_w, &orig_h) == 0) {
        fclose(f);
        
        static bool s_notified_resize = false;
        if ((orig_w != 64 || orig_h != 64) && !s_notified_resize) {
            printf("[AUTO-CONVERT] Raw PGM (%dx%d) automatically downsampled to 64x64 SNN input.\n", orig_w, orig_h);
            s_notified_resize = true;
        }

        /* Generate Bernoulli rate-coded spike stream across 25 timesteps */
        srand(42);
        for (int t = 0; t < SNN_TIMESTEPS; t++) {
            uint8_t *frame = spike_stream_out + (t * SNN_BYTES_PER_STEP);
            memset(frame, 0, SNN_BYTES_PER_STEP);
            for (int i = 0; i < SNN_SPIKES_PER_STEP; i++) {
                uint8_t intensity = gray_64x64[i];
                /* Spike if random value [0..255] < intensity */
                if ((rand() % 256) < intensity) {
                    int byte_idx = i / 8;
                    int bit_idx  = 7 - (i % 8);
                    frame[byte_idx] |= (1 << bit_idx);
                }
            }
        }
        return 0;
    }

    /* Case 4: Exact 4096 bytes (4096 raw pixels 0/1 or grayscale) */
    if (st.st_size == (off_t)SNN_SPIKES_PER_STEP) {
        fseek(f, 0, SEEK_SET);
        uint8_t pixels[SNN_SPIKES_PER_STEP];
        if (fread(pixels, 1, SNN_SPIKES_PER_STEP, f) != SNN_SPIKES_PER_STEP) {
            fclose(f);
            return -6;
        }
        fclose(f);

        uint8_t packed_frame[SNN_BYTES_PER_STEP];
        pack_64x64_to_512B(pixels, packed_frame);

        for (int t = 0; t < SNN_TIMESTEPS; t++) {
            memcpy(spike_stream_out + (t * SNN_BYTES_PER_STEP), packed_frame, SNN_BYTES_PER_STEP);
        }
        return 0;
    }

    /* Case 5: Text / ASCII file of 0s and 1s */
    fseek(f, 0, SEEK_SET);
    int bit_count = 0;
    uint8_t text_spikes[SNN_SPIKES_PER_STEP];
    memset(text_spikes, 0, SNN_SPIKES_PER_STEP);

    char token[64];
    while (bit_count < SNN_SPIKES_PER_STEP && fscanf(f, "%63s", token) == 1) {
        for (size_t i = 0; i < strlen(token) && bit_count < SNN_SPIKES_PER_STEP; i++) {
            if (token[i] == '0' || token[i] == '1') {
                text_spikes[bit_count++] = (token[i] == '1') ? 1 : 0;
            }
        }
    }
    fclose(f);

    if (bit_count == SNN_SPIKES_PER_STEP) {
        uint8_t frame[SNN_BYTES_PER_STEP];
        pack_64x64_to_512B(text_spikes, frame);
        for (int t = 0; t < SNN_TIMESTEPS; t++) {
            memcpy(spike_stream_out + (t * SNN_BYTES_PER_STEP), frame, SNN_BYTES_PER_STEP);
        }
        return 0;
    }

    return -7; /* Unrecognized file format */
}

int file_save_spikes_bin(const char *filepath, const uint8_t *spike_stream, size_t total_bytes) {
    if (!filepath || !spike_stream || total_bytes != SNN_TOTAL_SPIKE_BYTES) {
        return -1;
    }
    FILE *f = fopen(filepath, "wb");
    if (!f) return -2;
    size_t written = fwrite(spike_stream, 1, total_bytes, f);
    fclose(f);
    return (written == total_bytes) ? 0 : -3;
}

void file_generate_scenario(uint8_t *spike_stream_out, spike_scenario_t scenario) {
    if (!spike_stream_out) return;

    memset(spike_stream_out, 0, SNN_TOTAL_SPIKE_BYTES);
    uint8_t pixels[64][64];
    memset(pixels, 0, sizeof(pixels));

    srand(54321);

    switch (scenario) {
        case SCENARIO_OBSTACLE_COLLISION:
            /* Prominent central obstacle in 64x64 field */
            for (int r = 18; r < 46; r++) {
                for (int c = 18; c < 46; c++) {
                    pixels[r][c] = 1;
                }
            }
            break;

        case SCENARIO_SIDE_OBSTACLE:
            /* Obstacle on the side */
            for (int r = 16; r < 48; r++) {
                for (int c = 0; c < 24; c++) {
                    pixels[r][c] = 1;
                }
            }
            break;

        case SCENARIO_ALL_ONES:
            memset(pixels, 1, sizeof(pixels));
            break;

        case SCENARIO_ALL_ZEROS:
            memset(pixels, 0, sizeof(pixels));
            break;

        case SCENARIO_CLEAR_PATH:
        default:
            /* Very sparse background noise (< 1% activity) */
            for (int r = 0; r < 64; r++) {
                for (int c = 0; c < 64; c++) {
                    pixels[r][c] = (rand() % 100 == 0) ? 1 : 0;
                }
            }
            break;
    }

    /* Convert 64x64 grid to 512 bytes */
    uint8_t frame[SNN_BYTES_PER_STEP];
    pack_64x64_to_512B((const uint8_t *)pixels, frame);

    /* Fill all 25 timesteps */
    for (int t = 0; t < SNN_TIMESTEPS; t++) {
        memcpy(spike_stream_out + (t * SNN_BYTES_PER_STEP), frame, SNN_BYTES_PER_STEP);
    }
}

void file_render_ascii_image(const uint8_t *frame_512B) {
    if (!frame_512B) return;

    printf("\n  +----------------------------------------------------------------+\n");
    printf("  |                  64x64 INPUT SENSOR / IMAGE FRAME              |\n");
    printf("  +----------------------------------------------------------------+\n");

    /* Render 64 columns downsampled by 2 vertically (32 rows) for a square terminal view */
    for (int r = 0; r < 64; r += 2) {
        printf("  |");
        for (int c = 0; c < 64; c++) {
            int idx1 = r * 64 + c;
            int idx2 = (r + 1) * 64 + c;

            int bit1 = (frame_512B[idx1 / 8] >> (7 - (idx1 % 8))) & 1;
            int bit2 = (frame_512B[idx2 / 8] >> (7 - (idx2 % 8))) & 1;

            if (bit1 && bit2) {
                printf("#");
            } else if (bit1 || bit2) {
                printf("+");
            } else {
                printf(" ");
            }
        }
        printf("|\n");
    }
    printf("  +----------------------------------------------------------------+\n\n");
}

bool file_is_directory(const char *path) {
    if (!path) return false;
    struct stat st;
    if (stat(path, &st) != 0) return false;
    return S_ISDIR(st.st_mode);
}

static bool has_supported_image_ext(const char *name) {
    if (!name) return false;
    const char *dot = strrchr(name, '.');
    if (!dot) return false;
    if (strcasecmp(dot, ".pgm") == 0 ||
        strcasecmp(dot, ".ppm") == 0 ||
        strcasecmp(dot, ".pbm") == 0 ||
        strcasecmp(dot, ".bin") == 0 ||
        strcasecmp(dot, ".dat") == 0 ||
        strcasecmp(dot, ".txt") == 0) {
        return true;
    }
    return false;
}

static int compare_str_pointers(const void *a, const void *b) {
    const char *str1 = *(const char **)a;
    const char *str2 = *(const char **)b;
    return strcmp(str1, str2);
}

#include <dirent.h>

int file_scan_directory(const char *dirpath, char ***filepaths_out, int *count_out) {
    if (!dirpath || !filepaths_out || !count_out) return -1;
    *filepaths_out = NULL;
    *count_out = 0;

    DIR *d = opendir(dirpath);
    if (!d) return -2;

    int capacity = 128;
    int count = 0;
    char **list = (char **)malloc(capacity * sizeof(char *));
    if (!list) {
        closedir(d);
        return -3;
    }

    struct dirent *entry;
    while ((entry = readdir(d)) != NULL) {
        if (entry->d_name[0] == '.') continue; /* Skip hidden files and . / .. */
        if (has_supported_image_ext(entry->d_name)) {
            char fullpath[1024];
            size_t dlen = strlen(dirpath);
            if (dlen > 0 && dirpath[dlen - 1] == '/') {
                snprintf(fullpath, sizeof(fullpath), "%s%s", dirpath, entry->d_name);
            } else {
                snprintf(fullpath, sizeof(fullpath), "%s/%s", dirpath, entry->d_name);
            }

            if (count >= capacity) {
                capacity *= 2;
                char **new_list = (char **)realloc(list, capacity * sizeof(char *));
                if (!new_list) {
                    file_free_scanned_list(list, count);
                    closedir(d);
                    return -4;
                }
                list = new_list;
            }

            list[count] = strdup(fullpath);
            count++;
        }
    }
    closedir(d);

    if (count > 0) {
        qsort(list, count, sizeof(char *), compare_str_pointers);
    }

    *filepaths_out = list;
    *count_out = count;
    return 0;
}

void file_free_scanned_list(char **filepaths, int count) {
    if (!filepaths) return;
    for (int i = 0; i < count; i++) {
        if (filepaths[i]) free(filepaths[i]);
    }
    free(filepaths);
}
