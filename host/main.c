#include "snn_protocol.h"
#include "eth_comm.h"
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
#include <stdbool.h>

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
    printf("   SNN ACCELERATOR - FPGA RAW ETHERNET INFERENCE & CONTROLLER          \n");
    printf("=======================================================================\n");
    printf(ANSI_COLOR_RESET);
}

static void print_usage(const char *prog_name) {
    printf("\nUsage: %s [OPTIONS]\n\n", prog_name);
    printf("Options:\n");
    printf("  -e, --ifname <interface>  Network interface name (default: %s)\n", DEFAULT_IFNAME);
    printf("  -m, --mac <mac_addr>      FPGA Target MAC address (default: %s)\n", DEFAULT_FPGA_MAC_STR);
    printf("  -w, --weights <file>      Write weights file to FPGA BRAM\n");
    printf("  -i, --image <file|dir>    Single image/spike file OR folder of images (Batch Mode)\n");
    printf("  -d, --dir <dir>           Directory of images to process in batch\n");
    printf("  -s, --sim                 Run software SNN simulator (offline / verification)\n");
    printf("  -o, --csv <file>          Export batch inference results to CSV file\n");
    printf("  -n, --limit <count>       Limit batch run to first N images\n");
    printf("  -a, --ascii               Show ASCII art visualization of input frame\n");
    printf("  -p, --ping                Ping FPGA accelerator over Ethernet\n");
    printf("  -g, --generate            Generate sample weight and image datasets in ../data\n");
    printf("  -t, --test                Run full software self-tests and test vectors\n");
    printf("  -v, --verbose             Verbose logging for every timestep\n");
    printf("  -h, --help                Display this help information\n\n");
    printf("Examples:\n");
    printf("  1. Ping FPGA over Ethernet:\n");
    printf("       sudo %s --ifname eno1 --ping\n\n", prog_name);
    printf("  2. Single Image Inference on FPGA:\n");
    printf("       sudo %s --ifname eno1 --weights ../data/weights.bin --image ../data/sample_collision_img.bin\n\n", prog_name);
    printf("  3. Batch Directory Inference on FPGA:\n");
    printf("       sudo %s --ifname eno1 --weights ../data/weights.bin --dir ../data/ --csv results.csv\n\n", prog_name);
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
        printf("  FPGA Ethernet Return  : 0x%02X\n", (uint8_t)hw_result);
    }
    printf("  Hardware LED State    : LED[0]=Heartbeat, LED[1]=RX, LED[2]=TX, LED[3]=PHY Ready\n");
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

    printf("\nSelf-Test Summary: %d Passed, %d Failed.\n\n", passed, failed);
    return (failed == 0) ? 0 : 1;
}

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
    char ifname[32] = DEFAULT_IFNAME;
    char fpga_mac[32] = DEFAULT_FPGA_MAC_STR;
    char weights_file[256] = "";
    char image_path[512] = "";
    char csv_output[256] = "";
    int limit_count = 0;
    bool sim_only = false;
    bool show_ascii = false;
    bool verbose = false;
    bool do_ping = false;

    static struct option long_options[] = {
        {"ifname",      required_argument, 0, 'e'},
        {"mac",         required_argument, 0, 'm'},
        {"weights",     required_argument, 0, 'w'},
        {"image",       required_argument, 0, 'i'},
        {"dir",         required_argument, 0, 'd'},
        {"batch",       required_argument, 0, 'd'},
        {"csv",         required_argument, 0, 'o'},
        {"limit",       required_argument, 0, 'n'},
        {"sim",         no_argument,       0, 's'},
        {"ping",        no_argument,       0, 'p'},
        {"ascii",       no_argument,       0, 'a'},
        {"test",        no_argument,       0, 't'},
        {"verbose",     no_argument,       0, 'v'},
        {"help",        no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "e:m:w:i:d:o:n:spatvh", long_options, NULL)) != -1) {
        switch (opt) {
            case 'e':
                safe_strcpy(ifname, optarg, sizeof(ifname));
                break;
            case 'm':
                safe_strcpy(fpga_mac, optarg, sizeof(fpga_mac));
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
            case 'p':
                do_ping = true;
                break;
            case 'a':
                show_ascii = true;
                break;
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

    if (do_ping) {
        eth_handle_t eth;
        printf("[INFO] Initializing raw Ethernet socket on %s to FPGA %s...\n", ifname, fpga_mac);
        if (eth_init(&eth, ifname, fpga_mac) != 0) {
            fprintf(stderr, ANSI_COLOR_RED "[ERROR] Could not initialize raw Ethernet interface. Run with sudo.\n" ANSI_COLOR_RESET);
            return 1;
        }
        int w_loaded = 0, f_empty = 0;
        printf("[PING] Sending Ping packet to FPGA...\n");
        if (eth_ping(&eth, &w_loaded, &f_empty, 1000) == 0) {
            printf(ANSI_COLOR_GREEN "[PONG RECEIVED] FPGA is ONLINE and reachable!\n" ANSI_COLOR_RESET);
            printf("  Weights Loaded : %s\n", w_loaded ? "YES" : "NO");
            printf("  FIFO State     : %s\n", f_empty ? "EMPTY" : "OCCUPIED");
        } else {
            fprintf(stderr, ANSI_COLOR_RED "[TIMEOUT] No Pong received from FPGA. Check cable/interface.\n" ANSI_COLOR_RESET);
        }
        eth_close(&eth);
        return 0;
    }

    if (weights_file[0] == '\0' && image_path[0] == '\0') {
        printf(ANSI_COLOR_YELLOW "No weights or image/dir path specified.\n" ANSI_COLOR_RESET);
        print_usage(argv[0]);
        return 0;
    }

    int16_t weights[SNN_NUM_NEURONS];
    bool have_weights = false;

    if (weights_file[0] != '\0') {
        printf("[INFO] Loading weights from '%s'...\n", weights_file);
        if (file_load_weights(weights_file, weights, SNN_NUM_NEURONS) != 0) {
            fprintf(stderr, ANSI_COLOR_RED "[ERROR] Failed to load weights from '%s'.\n" ANSI_COLOR_RESET, weights_file);
            return 1;
        }
        have_weights = true;
        printf(ANSI_COLOR_GREEN "[OK] Successfully loaded %d weights.\n" ANSI_COLOR_RESET, SNN_NUM_NEURONS);
    } else {
        printf(ANSI_COLOR_YELLOW "[NOTE] Using built-in obstacle detector weights.\n" ANSI_COLOR_RESET);
        file_generate_weights(weights, SNN_NUM_NEURONS, WEIGHT_PATTERN_OBSTACLE_DETECTOR);
        have_weights = true;
    }

    bool is_batch_dir = (image_path[0] != '\0' && file_is_directory(image_path));

    /* Single Image Mode */
    if (!is_batch_dir) {
        uint8_t spike_stream[SNN_TOTAL_SPIKE_BYTES];
        bool have_image = false;

        if (image_path[0] != '\0') {
            printf("[INFO] Loading image/spikes from '%s'...\n", image_path);
            if (file_load_spikes_or_image(image_path, spike_stream) != 0) {
                fprintf(stderr, ANSI_COLOR_RED "[ERROR] Failed to load image from '%s'.\n" ANSI_COLOR_RESET, image_path);
                return 1;
            }
            have_image = true;
            printf(ANSI_COLOR_GREEN "[OK] Successfully loaded image/spikes (%d bytes).\n" ANSI_COLOR_RESET, SNN_TOTAL_SPIKE_BYTES);

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
            printf(ANSI_COLOR_CYAN "[SIMULATION COMPLETE] (Offline mode)\n" ANSI_COLOR_RESET);
            display_collision_banner(sim_res.collision, sim_res.total_spikes, -1);
            return 0;
        }

        eth_handle_t eth;
        printf("[INFO] Connecting to FPGA on %s (MAC %s)...\n", ifname, fpga_mac);
        if (eth_init(&eth, ifname, fpga_mac) != 0) {
            fprintf(stderr, ANSI_COLOR_YELLOW "[WARNING] Could not open raw socket. Run with sudo.\n" ANSI_COLOR_RESET);
            if (have_weights && have_image) {
                display_collision_banner(sim_res.collision, sim_res.total_spikes, -1);
            }
            return 2;
        }

        if (weights_file[0] != '\0') {
            printf(ANSI_COLOR_CYAN "[STEP 1/2] Writing weights to FPGA BRAM via Ethernet...\n" ANSI_COLOR_RESET);
            int ret = eth_send_weights(&eth, weights, SNN_NUM_NEURONS, weights_progress_callback);
            if (ret == 0) {
                printf(ANSI_COLOR_GREEN "[SUCCESS] Weights verified in FPGA Block RAM.\n\n" ANSI_COLOR_RESET);
            } else {
                fprintf(stderr, ANSI_COLOR_RED "[ERROR] Weight write failed (%d).\n" ANSI_COLOR_RESET, ret);
                eth_close(&eth);
                return 3;
            }
        }

        if (have_image) {
            printf(ANSI_COLOR_CYAN "[STEP 2/2] Streaming spike frames to FPGA over Ethernet...\n" ANSI_COLOR_RESET);
            int hw_res = -1;
            int ret = eth_send_spikes(&eth, spike_stream, SNN_TOTAL_SPIKE_BYTES, &hw_res, spikes_progress_callback);
            if (ret == 0) {
                printf(ANSI_COLOR_GREEN "[SUCCESS] Inference completed.\n" ANSI_COLOR_RESET);
                int final_col = (hw_res == 1) ? 1 : (hw_res == 0) ? 0 : sim_res.collision;
                display_collision_banner(final_col, sim_res.total_spikes, hw_res);
            } else {
                fprintf(stderr, ANSI_COLOR_RED "[ERROR] Spike streaming failed (%d).\n" ANSI_COLOR_RESET, ret);
                eth_close(&eth);
                return 4;
            }
        }

        eth_close(&eth);
        return 0;
    }

    /* Batch Directory Mode */
    char **file_list = NULL;
    int total_files = 0;
    file_scan_directory(image_path, &file_list, &total_files);
    if (limit_count > 0 && limit_count < total_files) total_files = limit_count;

    printf(ANSI_COLOR_GREEN "[OK] Found %d images for batch processing.\n" ANSI_COLOR_RESET, total_files);

    eth_handle_t eth;
    bool eth_connected = false;
    if (!sim_only) {
        if (eth_init(&eth, ifname, fpga_mac) == 0) {
            eth_connected = true;
            printf(ANSI_COLOR_CYAN "[STEP 1/2] Loading weights ONCE into FPGA BRAM...\n" ANSI_COLOR_RESET);
            eth_send_weights(&eth, weights, SNN_NUM_NEURONS, weights_progress_callback);
        } else {
            printf(ANSI_COLOR_YELLOW "[WARNING] Fallback to software simulation.\n" ANSI_COLOR_RESET);
            sim_only = true;
        }
    }

    FILE *csv_file = (csv_output[0] != '\0') ? fopen(csv_output, "w") : NULL;
    if (csv_file) {
        fprintf(csv_file, "index,filename,prediction,class_name,latency_ms\n");
    }

    printf(ANSI_COLOR_CYAN "[STEP 2/2] Running Batch Inference on %d images...\n" ANSI_COLOR_RESET, total_files);
    uint8_t spike_stream[SNN_TOTAL_SPIKE_BYTES];
    int collisions = 0, clears = 0;
    double start_t = get_time_ms();

    for (int i = 0; i < total_files; i++) {
        double t0 = get_time_ms();
        file_load_spikes_or_image(file_list[i], spike_stream);

        int pred = 0;
        if (eth_connected) {
            int hw_res = -1;
            eth_send_spikes(&eth, spike_stream, SNN_TOTAL_SPIKE_BYTES, &hw_res, NULL);
            pred = (hw_res == 1) ? 1 : 0;
        } else {
            snn_sim_result_t sim_res;
            snn_sim_run(NULL, weights, spike_stream, &sim_res);
            pred = sim_res.collision;
        }

        double lat = get_time_ms() - t0;
        if (pred == 1) collisions++; else clears++;

        const char *bn = strrchr(file_list[i], '/');
        bn = bn ? bn + 1 : file_list[i];

        if (i < 10 || i >= total_files - 5) {
            printf("  [%3d/%3d] %-30s -> %s (%.1f ms)\n",
                   i + 1, total_files, bn,
                   pred ? ANSI_COLOR_RED "COLLISION" ANSI_COLOR_RESET : ANSI_COLOR_GREEN "CLEAR" ANSI_COLOR_RESET,
                   lat);
        }

        if (csv_file) {
            fprintf(csv_file, "%d,\"%s\",%d,\"%s\",%.2f\n",
                    i + 1, bn, pred, pred ? "COLLISION" : "CLEAR", lat);
        }
    }

    double total_ms = get_time_ms() - start_t;
    printf("\n=== BATCH SUMMARY ===\n");
    printf("  Total: %d images in %.2f s (%.1f FPS)\n", total_files, total_ms / 1000.0, total_files / (total_ms / 1000.0));
    printf("  Collisions: %d, Clear: %d\n\n", collisions, clears);

    if (csv_file) fclose(csv_file);
    if (eth_connected) eth_close(&eth);
    file_free_scanned_list(file_list, total_files);

    return 0;
}
