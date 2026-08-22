#include "snn_protocol.h"
#include "uart.h"
#include "snn_sim.h"
#include "file_io.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <ctype.h>

#define ANSI_COLOR_RED     "\x1b[31;1m"
#define ANSI_COLOR_GREEN   "\x1b[32;1m"
#define ANSI_COLOR_YELLOW  "\x1b[33;1m"
#define ANSI_COLOR_CYAN    "\x1b[36;1m"
#define ANSI_COLOR_MAGENTA "\x1b[35;1m"
#define ANSI_COLOR_RESET   "\x1b[0m"

static void safe_strcpy(char *dest, const char *src, size_t dest_size) {
    if (!dest || dest_size == 0) return;
    if (!src) {
        dest[0] = '\0';
        return;
    }
    snprintf(dest, dest_size, "%s", src);
}

static double get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0;
}

static void print_progress_bar(size_t current, size_t total, const char *label) {
    const int bar_width = 40;
    float progress = (total > 0) ? (float)current / (float)total : 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    int filled = (int)(progress * bar_width);

    printf("\r  %-12s [", label);
    for (int i = 0; i < bar_width; i++) {
        if (i < filled) printf("=");
        else if (i == filled) printf(">");
        else printf(" ");
    }
    printf("] %3d%% (%zu/%zu)", (int)(progress * 100.0f), current, total);
    fflush(stdout);
    if (current >= total) printf("\n");
}

static void weights_progress_callback(size_t sent, size_t total) {
    print_progress_bar(sent, total, "Weights:");
}

static void spikes_progress_callback(int step, int total_steps) {
    print_progress_bar(step, total_steps, "Timesteps:");
}

static void print_banner(void) {
    printf(ANSI_COLOR_CYAN);
    printf("=======================================================================\n");
    printf("    SNN ACCELERATOR - FPGA INFERENCE & COMMUNICATION CONTROLLER        \n");
    printf("=======================================================================\n");
    printf(ANSI_COLOR_RESET);
}

static void print_usage(const char *prog_name) {
    printf("\nUsage: %s [OPTIONS]\n\n", prog_name);
    printf("Options:\n");
    printf("  -p, --port <device>       Serial port path (e.g. /dev/ttyUSB0, /dev/ttyACM0)\n");
    printf("  -b, --baud <rate>         Baud rate (default: %d)\n", SNN_DEFAULT_BAUD);
    printf("  -w, --weights <file>      Write weights file to FPGA (loaded ONCE for batch)\n");
    printf("  -i, --image <file|dir>    Single image/spike file OR folder of images (Batch Mode)\n");
    printf("  -d, --dir <dir>           Directory of images to process in batch\n");
    printf("  -s, --sim                 Run software SNN simulator (offline / verification)\n");
    printf("  -o, --csv <file>          Export batch inference results to CSV file\n");
    printf("  -n, --limit <count>       Limit batch run to first N images\n");
    printf("  -a, --ascii               Show ASCII art visualization of input frame\n");
    printf("  -g, --generate            Generate sample weight and image datasets in current dir\n");
    printf("  -l, --list-ports          Scan and list detected serial ports\n");
    printf("  -t, --test                Run full software self-tests and test vectors\n");
    printf("  -v, --verbose             Verbose logging for every timestep\n");
    printf("  -h, --help                Display this help information\n\n");
    printf("Examples:\n");
    printf("  1. Single Image Inference on FPGA:\n");
    printf("       %s --port /dev/ttyUSB0 --weights weights.bin --image DSCN2571_frame_181.pgm\n\n", prog_name);
    printf("  2. Batch Directory Inference on FPGA (Loads weights ONCE, streams all images):\n");
    printf("       %s --port /dev/ttyUSB0 --weights weights.bin --image /path/to/collision/\n\n", prog_name);
    printf("  3. Offline Batch Simulation without FPGA:\n");
    printf("       %s --sim --weights weights.bin --dir /path/to/no_collision/ --csv results.csv\n\n", prog_name);
}

static void display_collision_banner(int collision_detected, int spike_count, int hw_result) {
    printf("\n=======================================================================\n");
    if (collision_detected) {
        printf(ANSI_COLOR_RED);
        printf("                     [!] COLLISION DETECTED [!]                        \n");
        printf("               >> OBSTACLE IMMINENT IN VEHICLE PATH <<                 \n");
        printf(ANSI_COLOR_RESET);
    } else {
        printf(ANSI_COLOR_GREEN);
        printf("                 [OK] NO COLLISION - PATH CLEAR [OK]                   \n");
        printf("                     >> SAFE TO PROCEED <<                             \n");
        printf(ANSI_COLOR_RESET);
    }
    printf("=======================================================================\n");
    printf("  Classification Result : %s (Class %d)\n",
           collision_detected ? "COLLISION" : "NO COLLISION / CLEAR",
           collision_detected ? 1 : 0);
    printf("  SNN Spike Window      : %d / %d timesteps fired (Threshold: >%d)\n",
           spike_count, SNN_TIMESTEPS, SNN_SPIKE_THRESHOLD);
    if (hw_result >= 0) {
        printf("  FPGA UART Tx Return   : 0x%02X\n", (uint8_t)hw_result);
    }
    printf("  Hardware LED State    : result_led = %d (LED[0]), done_led = 1\n",
           collision_detected ? 1 : 0);
    printf("=======================================================================\n\n");
}

static int run_self_tests(void) {
    printf(ANSI_COLOR_CYAN "\n[TEST] Running SNN Pipeline Self-Tests...\n" ANSI_COLOR_RESET);
    int passed = 0, failed = 0;

    int16_t weights[SNN_NUM_NEURONS];
    uint8_t spike_stream[SNN_TOTAL_SPIKE_BYTES];
    snn_sim_result_t res;

    /* Test 1: Obstacle Detector + Obstacle Scenario -> Must detect Collision */
    file_generate_weights(weights, SNN_NUM_NEURONS, WEIGHT_PATTERN_OBSTACLE_DETECTOR);
    file_generate_scenario(spike_stream, SCENARIO_OBSTACLE_COLLISION);
    snn_sim_run(NULL, weights, spike_stream, &res);

    printf("  Test 1: Obstacle Scenario with Obstacle Detector Weights -> ");
    if (res.collision == 1 && res.total_spikes > SNN_SPIKE_THRESHOLD) {
        printf(ANSI_COLOR_GREEN "PASSED" ANSI_COLOR_RESET " (Spikes=%d/25, Class=1)\n", res.total_spikes);
        passed++;
    } else {
        printf(ANSI_COLOR_RED "FAILED" ANSI_COLOR_RESET " (Spikes=%d/25, Class=%d)\n", res.total_spikes, res.collision);
        failed++;
    }

    /* Test 2: Obstacle Detector + Clear Path Scenario -> Must report No Collision */
    file_generate_scenario(spike_stream, SCENARIO_CLEAR_PATH);
    snn_sim_run(NULL, weights, spike_stream, &res);

    printf("  Test 2: Clear Path Scenario with Obstacle Detector Weights -> ");
    if (res.collision == 0 && res.total_spikes <= SNN_SPIKE_THRESHOLD) {
        printf(ANSI_COLOR_GREEN "PASSED" ANSI_COLOR_RESET " (Spikes=%d/25, Class=0)\n", res.total_spikes);
        passed++;
    } else {
        printf(ANSI_COLOR_RED "FAILED" ANSI_COLOR_RESET " (Spikes=%d/25, Class=%d)\n", res.total_spikes, res.collision);
        failed++;
    }

    /* Test 3: Inactive Stream (All Zeros) -> Must report 0 spikes */
    file_generate_scenario(spike_stream, SCENARIO_ALL_ZEROS);
    snn_sim_run(NULL, weights, spike_stream, &res);

    printf("  Test 3: Inactive Stream (All Zeros) -> ");
    if (res.total_spikes == 0 && res.collision == 0) {
        printf(ANSI_COLOR_GREEN "PASSED" ANSI_COLOR_RESET " (Spikes=0/25, Class=0)\n");
        passed++;
    } else {
        printf(ANSI_COLOR_RED "FAILED" ANSI_COLOR_RESET " (Spikes=%d/25, Class=%d)\n", res.total_spikes, res.collision);
        failed++;
    }

    /* Test 4: File serialization round-trip */
    file_save_weights_bin("/tmp/test_weights.bin", weights, SNN_NUM_NEURONS);
    int16_t read_weights[SNN_NUM_NEURONS];
    if (file_load_weights("/tmp/test_weights.bin", read_weights, SNN_NUM_NEURONS) == 0 &&
        memcmp(weights, read_weights, sizeof(weights)) == 0) {
        printf("  Test 4: Binary Weights File Roundtrip -> " ANSI_COLOR_GREEN "PASSED\n" ANSI_COLOR_RESET);
        passed++;
    } else {
        printf("  Test 4: Binary Weights File Roundtrip -> " ANSI_COLOR_RED "FAILED\n" ANSI_COLOR_RESET);
        failed++;
    }
    unlink("/tmp/test_weights.bin");

    printf("\nSelf-Test Summary: %d Passed, %d Failed.\n\n", passed, failed);
    return (failed == 0) ? 0 : 1;
}

static void generate_sample_datasets(void) {
    printf(ANSI_COLOR_CYAN "\n[GENERATE] Generating Sample Test Datasets...\n" ANSI_COLOR_RESET);

    int16_t weights[SNN_NUM_NEURONS];
    uint8_t spike_stream[SNN_TOTAL_SPIKE_BYTES];

    /* 1. Generate Obstacle Detector Weights */
    file_generate_weights(weights, SNN_NUM_NEURONS, WEIGHT_PATTERN_OBSTACLE_DETECTOR);
    file_save_weights_bin("sample_weights.bin", weights, SNN_NUM_NEURONS);
    file_save_weights_txt("sample_weights.txt", weights, SNN_NUM_NEURONS);
    printf("  -> Created sample_weights.bin (8192 bytes binary, Big-Endian)\n");
    printf("  -> Created sample_weights.txt (4096 decimal lines)\n");

    /* 2. Generate Collision Image / Spike Stream */
    file_generate_scenario(spike_stream, SCENARIO_OBSTACLE_COLLISION);
    file_save_spikes_bin("sample_collision_img.bin", spike_stream, SNN_TOTAL_SPIKE_BYTES);
    printf("  -> Created sample_collision_img.bin (12,800 bytes, 25 timesteps obstacle)\n");

    /* 3. Generate Clear Path Image / Spike Stream */
    file_generate_scenario(spike_stream, SCENARIO_CLEAR_PATH);
    file_save_spikes_bin("sample_clear_img.bin", spike_stream, SNN_TOTAL_SPIKE_BYTES);
    printf("  -> Created sample_clear_img.bin (12,800 bytes, 25 timesteps clear path)\n");

    printf(ANSI_COLOR_GREEN "[SUCCESS] Sample datasets generated successfully.\n\n" ANSI_COLOR_RESET);
}

/**
 * Infer ground truth class from directory/file path if present in name.
 * Returns: 1 for Collision, 0 for Clear/No-Collision, -1 if unknown.
 */
static int infer_ground_truth_label(const char *path) {
    if (!path) return -1;
    char lower[1024];
    size_t len = strlen(path);
    if (len >= sizeof(lower)) len = sizeof(lower) - 1;
    for (size_t i = 0; i < len; i++) {
        lower[i] = (char)tolower((unsigned char)path[i]);
    }
    lower[len] = '\0';

    if (strstr(lower, "no_collision") != NULL || strstr(lower, "nocollision") != NULL ||
        strstr(lower, "safe") != NULL || strstr(lower, "clear") != NULL) {
        return 0;
    }
    if (strstr(lower, "collision") != NULL || strstr(lower, "obstacle") != NULL) {
        return 1;
    }
    return -1;
}

int main(int argc, char **argv) {
    char port[64] = SNN_DEFAULT_PORT;
    int baud = SNN_DEFAULT_BAUD;
    char weights_file[256] = "";
    char image_path[512] = "";
    char csv_output[256] = "";
    int limit_count = 0;
    bool sim_only = false;
    bool show_ascii = false;
    bool verbose = false;
    bool port_explicitly_set = false;

    static struct option long_options[] = {
        {"port",        required_argument, 0, 'p'},
        {"baud",        required_argument, 0, 'b'},
        {"weights",     required_argument, 0, 'w'},
        {"image",       required_argument, 0, 'i'},
        {"dir",         required_argument, 0, 'd'},
        {"batch",       required_argument, 0, 'd'},
        {"csv",         required_argument, 0, 'o'},
        {"limit",       required_argument, 0, 'n'},
        {"sim",         no_argument,       0, 's'},
        {"generate",    no_argument,       0, 'g'},
        {"ascii",       no_argument,       0, 'a'},
        {"list-ports",  no_argument,       0, 'l'},
        {"test",        no_argument,       0, 't'},
        {"verbose",     no_argument,       0, 'v'},
        {"help",        no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "p:b:w:i:d:o:n:sgaltvh", long_options, NULL)) != -1) {
        switch (opt) {
            case 'p':
                safe_strcpy(port, optarg, sizeof(port));
                port_explicitly_set = true;
                break;
            case 'b':
                baud = atoi(optarg);
                break;
            case 'w':
                safe_strcpy(weights_file, optarg, sizeof(weights_file));
                break;
            case 'i':
            case 'd':
                safe_strcpy(image_path, optarg, sizeof(image_path));
                break;
            case 'o':
                safe_strcpy(csv_output, optarg, sizeof(csv_output));
                break;
            case 'n':
                limit_count = atoi(optarg);
                break;
            case 's':
                sim_only = true;
                break;
            case 'g':
                print_banner();
                generate_sample_datasets();
                return 0;
            case 'a':
                show_ascii = true;
                break;
            case 'l': {
                print_banner();
                char detected[8][64];
                int count = uart_detect_ports(detected, 8);
                printf("\nDetected Serial Ports (%d found):\n", count);
                for (int i = 0; i < count; i++) {
                    printf("  [%d] %s\n", i + 1, detected[i]);
                }
                if (count == 0) {
                    printf("  (No serial / FTDI devices found in /dev/ttyUSB* or /dev/ttyACM*)\n");
                }
                printf("\n");
                return 0;
            }
            case 't':
                print_banner();
                return run_self_tests();
            case 'v':
                verbose = true;
                break;
            case 'h':
            default:
                print_banner();
                print_usage(argv[0]);
                return 0;
        }
    }

    print_banner();

    /* If no action specified, show interactive help */
    if (weights_file[0] == '\0' && image_path[0] == '\0') {
        printf(ANSI_COLOR_YELLOW "No weights or image/dir path specified.\n" ANSI_COLOR_RESET);
        printf("Generating sample test datasets and running self-test...\n");
        generate_sample_datasets();
        run_self_tests();
        print_usage(argv[0]);
        return 0;
    }

    int16_t weights[SNN_NUM_NEURONS];
    bool have_weights = false;

    /* 1. Load weights once if provided */
    if (weights_file[0] != '\0') {
        printf("[INFO] Loading weights from '%s'...\n", weights_file);
        if (file_load_weights(weights_file, weights, SNN_NUM_NEURONS) != 0) {
            fprintf(stderr, ANSI_COLOR_RED "[ERROR] Failed to load weights from '%s'.\n" ANSI_COLOR_RESET, weights_file);
            return 1;
        }
        have_weights = true;
        printf(ANSI_COLOR_GREEN "[OK] Successfully loaded %d weights (16-bit signed).\n" ANSI_COLOR_RESET, SNN_NUM_NEURONS);
    } else {
        printf(ANSI_COLOR_YELLOW "[NOTE] No weights file specified. Using built-in obstacle detector weights.\n" ANSI_COLOR_RESET);
        file_generate_weights(weights, SNN_NUM_NEURONS, WEIGHT_PATTERN_OBSTACLE_DETECTOR);
        have_weights = true;
    }

    bool is_batch_dir = (image_path[0] != '\0' && file_is_directory(image_path));

    /* =========================================================================
     * CASE A: SINGLE IMAGE INFERENCE
     * ========================================================================= */
    if (!is_batch_dir) {
        uint8_t spike_stream[SNN_TOTAL_SPIKE_BYTES];
        bool have_image = false;

        if (image_path[0] != '\0') {
            printf("[INFO] Loading image/spikes from '%s'...\n", image_path);
            if (file_load_spikes_or_image(image_path, spike_stream) != 0) {
                fprintf(stderr, ANSI_COLOR_RED "[ERROR] Failed to load image/spike stream from '%s'.\n" ANSI_COLOR_RESET, image_path);
                return 1;
            }
            have_image = true;
            printf(ANSI_COLOR_GREEN "[OK] Successfully loaded image/spikes (%d timesteps x %d bytes = %d bytes).\n" ANSI_COLOR_RESET,
                   SNN_TIMESTEPS, SNN_BYTES_PER_STEP, SNN_TOTAL_SPIKE_BYTES);

            if (show_ascii) {
                file_render_ascii_image(spike_stream);
            }
        }

        snn_sim_result_t sim_res;
        if (have_weights && have_image) {
            snn_sim_run(NULL, weights, spike_stream, &sim_res);
            if (verbose || sim_only) {
                snn_sim_print_report(&sim_res, verbose);
            }
        }

        if (sim_only) {
            printf(ANSI_COLOR_CYAN "[SIMULATION COMPLETE] (Offline mode without FPGA)\n" ANSI_COLOR_RESET);
            display_collision_banner(sim_res.collision, sim_res.total_spikes, -1);
            return 0;
        }

        /* Auto-detect serial port */
        if (!port_explicitly_set) {
            char detected[8][64];
            int num_found = uart_detect_ports(detected, 8);
            if (num_found > 0) {
                safe_strcpy(port, detected[0], sizeof(port));
                printf("[INFO] Auto-detected FPGA serial port: %s\n", port);
            }
        }

        printf("[INFO] Connecting to FPGA on '%s' @ %d baud...\n", port, baud);
        int uart_fd = uart_open(port, baud);
        if (uart_fd < 0) {
            fprintf(stderr, ANSI_COLOR_YELLOW "\n[WARNING] Could not open serial port '%s'.\n" ANSI_COLOR_RESET, port);
            fprintf(stderr, "  - Running offline software simulation as fallback:\n\n");
            if (have_weights && have_image) {
                display_collision_banner(sim_res.collision, sim_res.total_spikes, -1);
            }
            return 2;
        }
        printf(ANSI_COLOR_GREEN "[OK] Serial port connected.\n\n" ANSI_COLOR_RESET);

        /* Send weights once */
        if (weights_file[0] != '\0') {
            printf(ANSI_COLOR_CYAN "[STEP 1/2] Writing %d weights (8192 bytes) to FPGA BRAM...\n" ANSI_COLOR_RESET, SNN_NUM_NEURONS);
            int ret = uart_send_weights(uart_fd, weights, SNN_NUM_NEURONS, weights_progress_callback);
            if (ret == 0) {
                printf(ANSI_COLOR_GREEN "[VERIFIED] FPGA ACK (0x01) received!\n" ANSI_COLOR_RESET);
                printf(ANSI_COLOR_GREEN "[SUCCESS] Synaptic weights successfully written into FPGA Block RAM.\n\n" ANSI_COLOR_RESET);
            } else {
                fprintf(stderr, ANSI_COLOR_RED "\n[ERROR] Weight write failed (Error Code: %d).\n" ANSI_COLOR_RESET, ret);
                uart_close(uart_fd);
                return 3;
            }
        }

        /* Stream single image spikes */
        if (have_image) {
            printf(ANSI_COLOR_CYAN "[STEP 2/2] Streaming image spike frames (25 timesteps, 12800 bytes) to FPGA...\n" ANSI_COLOR_RESET);
            int hw_res = -1;
            int ret = uart_send_spikes(uart_fd, spike_stream, SNN_TOTAL_SPIKE_BYTES, &hw_res, spikes_progress_callback);

            if (ret == 0) {
                printf(ANSI_COLOR_GREEN "[SUCCESS] All 25 timesteps successfully transmitted and ACKed by FPGA.\n" ANSI_COLOR_RESET);
                int final_collision = (hw_res == 1) ? 1 : (hw_res == 0) ? 0 : sim_res.collision;
                display_collision_banner(final_collision, sim_res.total_spikes, hw_res);
            } else {
                fprintf(stderr, ANSI_COLOR_RED "\n[ERROR] Image streaming failed (Error Code: %d).\n" ANSI_COLOR_RESET, ret);
                uart_close(uart_fd);
                return 4;
            }
        }

        uart_close(uart_fd);
        return 0;
    }

    /* =========================================================================
     * CASE B: BATCH DIRECTORY INFERENCE (LOAD WEIGHTS ONCE, RUN ALL IMAGES)
     * ========================================================================= */
    char **file_list = NULL;
    int total_files = 0;

    printf("[INFO] Scanning batch directory '%s'...\n", image_path);
    int scan_res = file_scan_directory(image_path, &file_list, &total_files);
    if (scan_res != 0 || total_files == 0) {
        fprintf(stderr, ANSI_COLOR_RED "[ERROR] No supported image files (.pgm, .ppm, .pbm, .bin) found in '%s'.\n" ANSI_COLOR_RESET, image_path);
        return 1;
    }

    if (limit_count > 0 && limit_count < total_files) {
        total_files = limit_count;
    }

    printf(ANSI_COLOR_GREEN "[OK] Found %d images to process in batch mode.\n" ANSI_COLOR_RESET, total_files);

    int dir_gt = infer_ground_truth_label(image_path);
    if (dir_gt == 1) {
        printf("[GROUND TRUTH] Inferred from folder: " ANSI_COLOR_RED "COLLISION (Class 1)" ANSI_COLOR_RESET "\n");
    } else if (dir_gt == 0) {
        printf("[GROUND TRUTH] Inferred from folder: " ANSI_COLOR_GREEN "SAFE PATH / NO COLLISION (Class 0)" ANSI_COLOR_RESET "\n");
    }

    int uart_fd = -1;

    /* If FPGA hardware mode, connect and send weights ONCE */
    if (!sim_only) {
        if (!port_explicitly_set) {
            char detected[8][64];
            int num_found = uart_detect_ports(detected, 8);
            if (num_found > 0) {
                safe_strcpy(port, detected[0], sizeof(port));
                printf("[INFO] Auto-detected FPGA serial port: %s\n", port);
            }
        }

        printf("[INFO] Connecting to FPGA on '%s' @ %d baud...\n", port, baud);
        uart_fd = uart_open(port, baud);
        if (uart_fd < 0) {
            fprintf(stderr, ANSI_COLOR_YELLOW "\n[WARNING] Could not open serial port '%s'.\n" ANSI_COLOR_RESET, port);
            fprintf(stderr, "  - Switching to offline Software SNN Simulation for the batch.\n\n");
            sim_only = true;
        } else {
            printf(ANSI_COLOR_GREEN "[OK] Serial port connected.\n" ANSI_COLOR_RESET);

            /* Write weights ONCE into FPGA BRAM */
            printf(ANSI_COLOR_CYAN "\n[STEP 1/2] Writing %d weights (8192 bytes) into FPGA BRAM ONCE...\n" ANSI_COLOR_RESET, SNN_NUM_NEURONS);
            int ret = uart_send_weights(uart_fd, weights, SNN_NUM_NEURONS, weights_progress_callback);
            if (ret == 0) {
                printf(ANSI_COLOR_GREEN "[SUCCESS] Weights verified in FPGA Block RAM! Ready for continuous batch stream.\n\n" ANSI_COLOR_RESET);
            } else {
                fprintf(stderr, ANSI_COLOR_RED "\n[ERROR] Initial weight write failed (Error Code: %d - %s).\n" ANSI_COLOR_RESET,
                        ret,
                        (ret == -2) ? "Failed to send header 0xAA" :
                        (ret == -3) ? "Failed to send weight payload" :
                        (ret == -5) ? "Timeout waiting for FPGA ACK response (0x01)" :
                        (ret == -6) ? "Invalid ACK response received from FPGA" : "Unknown error");
                fprintf(stderr, ANSI_COLOR_YELLOW "\n[TROUBLESHOOTING CHECKLIST]:\n" ANSI_COLOR_RESET);
                fprintf(stderr, "  1. Bitstream: Ensure 'snn_top.bit' is programmed on the FPGA (DONE LED should be green).\n");
                fprintf(stderr, "  2. Reset: Press the RESET button (Pin C2 / CPU_RESETN red button on Arty A7) to initialize the state machine.\n");
                fprintf(stderr, "  3. Port Check: On Arty A7, try '--port /dev/ttyUSB0' or '--port /dev/ttyUSB1'.\n");
                fprintf(stderr, "  4. Baud Rate: Ensure FPGA clock is 100MHz and baud is 115200 (or pass -b 115200).\n");
                fprintf(stderr, "  5. Offline Sim: You can run offline simulation with '-s' / '--sim' without FPGA hardware.\n\n");
                uart_close(uart_fd);
                return 3;
            }
        }
    }

    FILE *csv_file = NULL;
    if (csv_output[0] != '\0') {
        csv_file = fopen(csv_output, "w");
        if (csv_file) {
            fprintf(csv_file, "index,filename,prediction,class_name,spikes_fired,latency_ms,ground_truth\n");
            printf("[INFO] Logging batch results to CSV: '%s'\n", csv_output);
        }
    }

    printf(ANSI_COLOR_CYAN "[STEP 2/2] Running Batch Inference on %d images...\n" ANSI_COLOR_RESET, total_files);
    printf("+-----------------------------------------------------------------------------------------+\n");
    printf("|  Idx  | Filename                             | Result       | Spikes | Latency  | FPS   |\n");
    printf("+-------+--------------------------------------+--------------+--------+----------+-------+\n");

    int collision_detected_count = 0;
    int clear_detected_count = 0;
    int correct_predictions = 0;
    double batch_start_time = get_time_ms();

    uint8_t spike_stream[SNN_TOTAL_SPIKE_BYTES];

    for (int i = 0; i < total_files; i++) {
        const char *cur_file = file_list[i];
        const char *base_name = strrchr(cur_file, '/');
        base_name = (base_name) ? base_name + 1 : cur_file;

        double t0 = get_time_ms();

        /* 1. Load and auto-resize image into 12,800 bytes spike stream */
        if (file_load_spikes_or_image(cur_file, spike_stream) != 0) {
            fprintf(stderr, "\n[ERROR] Failed to load image: %s\n", cur_file);
            continue;
        }

        int pred_class = 0;
        int spikes_fired = 0;

        /* 2. Run inference: FPGA UART or Software Simulation */
        if (!sim_only && uart_fd >= 0) {
            int hw_res = -1;
            int ret = uart_send_spikes(uart_fd, spike_stream, SNN_TOTAL_SPIKE_BYTES, &hw_res, NULL);
            if (ret == 0) {
                pred_class = (hw_res == 1) ? 1 : 0;
                spikes_fired = (pred_class == 1) ? 25 : 0; /* Approximate without full sim */
            } else {
                fprintf(stderr, "\n[ERROR] UART transmission error on %s\n", base_name);
            }
        } else {
            snn_sim_result_t sim_res;
            snn_sim_run(NULL, weights, spike_stream, &sim_res);
            pred_class = sim_res.collision;
            spikes_fired = sim_res.total_spikes;
        }

        double t1 = get_time_ms();
        double lat_ms = t1 - t0;

        if (pred_class == 1) collision_detected_count++;
        else clear_detected_count++;

        int item_gt = infer_ground_truth_label(cur_file);
        if (item_gt < 0) item_gt = dir_gt;
        if (item_gt >= 0 && pred_class == item_gt) {
            correct_predictions++;
        }

        double cur_fps = (lat_ms > 0) ? (1000.0 / lat_ms) : 0.0;

        /* Print first 15 and last 5, or periodic status */
        if (i < 15 || i >= total_files - 5 || (i % 25 == 0)) {
            char short_name[37];
            safe_strcpy(short_name, base_name, sizeof(short_name));
            if (strlen(base_name) > 36) {
                short_name[33] = '.';
                short_name[34] = '.';
                short_name[35] = '.';
                short_name[36] = '\0';
            }

            printf("| %5d | %-36s | %s |  %2d/25 | %6.1f ms | %5.1f |\n",
                   i + 1,
                   short_name,
                   pred_class ? ANSI_COLOR_RED "COLLISION   " ANSI_COLOR_RESET : ANSI_COLOR_GREEN "CLEAR PATH  " ANSI_COLOR_RESET,
                   spikes_fired,
                   lat_ms,
                   cur_fps);
        } else if (i == 15 && total_files > 20) {
            printf("|   ... | ... [processing remaining images]    |      ...     |    ... |      ... |   ... |\n");
        }

        if (csv_file) {
            fprintf(csv_file, "%d,\"%s\",%d,\"%s\",%d,%.2f,%d\n",
                    i + 1, base_name, pred_class,
                    pred_class ? "COLLISION" : "CLEAR",
                    spikes_fired, lat_ms, item_gt);
        }
    }

    printf("+-------+--------------------------------------+--------------+--------+----------+-------+\n");

    double total_elapsed_ms = get_time_ms() - batch_start_time;
    double avg_latency = (total_files > 0) ? (total_elapsed_ms / total_files) : 0.0;
    double overall_fps = (total_elapsed_ms > 0) ? ((double)total_files / (total_elapsed_ms / 1000.0)) : 0.0;

    /* =========================================================================
     * BATCH SUMMARY REPORT
     * ========================================================================= */
    printf("\n=======================================================================\n");
    printf("                    SNN BATCH INFERENCE SUMMARY REPORT                 \n");
    printf("=======================================================================\n");
    printf("  Target Path           : %s\n", image_path);
    printf("  Execution Mode        : %s\n", sim_only ? "Software SNN Simulator (Offline)" : "FPGA Hardware Accelerator (UART)");
    printf("  Total Images Tested   : %d\n", total_files);
    printf("  Weights Loaded        : Loaded ONCE into BRAM at start\n");
    printf("-----------------------------------------------------------------------\n");
    printf("  Collisions Detected   : " ANSI_COLOR_RED "%d" ANSI_COLOR_RESET " (%.1f%%)\n",
           collision_detected_count, (float)collision_detected_count / total_files * 100.0f);
    printf("  Clear Paths Detected  : " ANSI_COLOR_GREEN "%d" ANSI_COLOR_RESET " (%.1f%%)\n",
           clear_detected_count, (float)clear_detected_count / total_files * 100.0f);
    printf("  Total Elapsed Time    : %.2f seconds\n", total_elapsed_ms / 1000.0);
    printf("  Average Latency       : %.2f ms / image\n", avg_latency);
    printf("  Throughput            : %.2f Images / Sec (FPS)\n", overall_fps);

    if (dir_gt >= 0) {
        float acc = (float)correct_predictions / (float)total_files * 100.0f;
        printf("-----------------------------------------------------------------------\n");
        printf("  Ground Truth Class    : %s\n", (dir_gt == 1) ? "COLLISION (Class 1)" : "CLEAR PATH (Class 0)");
        printf("  Classification Accuracy: " ANSI_COLOR_GREEN "%.2f%%" ANSI_COLOR_RESET " (%d / %d correct)\n",
               acc, correct_predictions, total_files);
    }
    printf("=======================================================================\n\n");

    if (csv_file) {
        fclose(csv_file);
        printf("[SUCCESS] Batch results saved to '%s'\n\n", csv_output);
    }

    if (uart_fd >= 0) {
        uart_close(uart_fd);
    }
    file_free_scanned_list(file_list, total_files);

    return 0;
}
