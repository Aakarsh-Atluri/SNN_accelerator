# SNN Accelerator - Host Inference & FPGA Communication Utility

This folder contains the complete C-based host software for interfacing with the **Spiking Neural Network (SNN) hardware accelerator** over UART.

It provides end-to-end functionality to:
1. **Write Synaptic Weights** (4096 $\times$ 16-bit signed weights = 8192 bytes) into the FPGA Block RAM via UART and verify reception via hardware ACK (`0x01`).
2. **Stream Input Images / Spike Frames** across 25 timesteps ($25 \times 512\text{ bytes} = 12,800\text{ bytes}$) with 512-byte chunked flow control and per-timestep ACK handshaking.
3. **Display Classification Results**: Instant visual feedback on whether a **Collision** (Class 1) or **No Collision / Clear Path** (Class 0) was detected, including FPGA LED states and spike window statistics.
4. **Bit-Accurate SNN Simulator**: Runs a golden reference software model of the LIF neuron and cascaded MAC accumulator for instant offline verification.

---

## 🛠️ Building the Program

Compile using `make`:

```bash
cd inference
make
```

### Build Targets:
- `make` : Builds the `snn_inference` executable.
- `make test` : Runs self-tests on the SNN simulation engine and parser.
- `make sample_data` : Generates sample weights (`sample_weights.bin`, `sample_weights.txt`) and image datasets (`sample_collision_img.bin`, `sample_clear_img.bin`).
- `make clean` : Cleans compiled objects and generated sample binaries.

---

## 🚀 Quick Start Examples

### 1. Generate Sample Datasets
```bash
./snn_inference --generate
```

### 2. Write Weights to FPGA
```bash
./snn_inference --port /dev/ttyUSB0 --weights sample_weights.bin
```
**Output:**
```text
[STEP 1/2] Writing 4096 weights (8192 bytes) to FPGA BRAM...
  Weights:     [========================================] 100% (8192/8192)
[VERIFIED] FPGA ACK (0x01) received!
[SUCCESS] Synaptic weights successfully written into FPGA Block RAM.
```

### 3. Run Inference on an Image
```bash
./snn_inference --port /dev/ttyUSB0 --weights sample_weights.bin --image sample_collision_img.bin --ascii
```
**Output:**
```text
=======================================================================
                     [!] COLLISION DETECTED [!]                        
               >> OBSTACLE IMMINENT IN VEHICLE PATH <<                 
=======================================================================
  Classification Result : COLLISION (Class 1)
  SNN Spike Window      : 25 / 25 timesteps fired (Threshold: >12)
  Hardware LED State    : result_led = 1 (LED[0]), done_led = 1
=======================================================================
```

### 4. Run Batch Inference on an Entire Folder of Raw PGM Images
Loads weights **ONCE** into FPGA Block RAM, then streams all images consecutively:
```bash
./snn_inference --port /dev/ttyUSB0 --weights weights.bin --dir /path/to/dataset/collision/ --csv batch_results.csv
```

### 5. Offline Software Simulation (Without FPGA Hardware)
```bash
# Single image
./snn_inference --sim -w weights.bin -i DSCN2571_frame_181.pgm -a -v

# Entire batch folder offline
./snn_inference --sim -w weights.bin --dir /path/to/dataset/collision/ --csv results.csv
```

---

## 📋 Command-Line Options

| Option | Flag | Description |
|---|---|---|
| `--port <device>` | `-p` | Serial device path (e.g. `/dev/ttyUSB0`, `/dev/ttyACM0`). Auto-detected if omitted. |
| `--baud <rate>` | `-b` | UART Baud rate (default: `115200`, 8N1). |
| `--weights <file>` | `-w` | Path to weights file (Binary 8192B or Text 4096 lines). Loaded **ONCE** at startup. |
| `--image <file\|dir>` | `-i` | Path to image file OR folder of images (Batch Mode). |
| `--dir <dir>` | `-d` | Path to directory of images to process in batch mode. |
| `--csv <file>` | `-o` | Export batch inference predictions and stats to CSV. |
| `--limit <count>` | `-n` | Process only first N images in batch directory. |
| `--sim` | `-s` | Run software SNN simulation (offline verification mode). |
| `--ascii` | `-a` | Render 64x64 ASCII art visualization of the input frame. |
| `--generate` | `-g` | Generate synthetic weights and image test vectors. |
| `--list-ports` | `-l` | Scan and list all available USB/FTDI serial devices. |
| `--test` | `-t` | Run built-in self-tests. |
| `--verbose` | `-v` | Display per-timestep MAC current, leak value, and membrane potential. |
| `--help` | `-h` | Display help screen. |

---

## 🔌 UART Protocol & Hardware Interfacing

```
   Host PC (snn_inference)                      FPGA (snn_top.v)
   -----------------------                      ----------------
1. Weight Loading:
   [0xAA Header] -----------------------------> fifo_uart_controller (RCV_WEIGHTS)
   [8192 Bytes Payload (BE)] -----------------> BRAM Write Port (addr 0..4095)
   [0xBB Ping] -------------------------------> fifo_uart_controller (IDLE -> SEND_ACK)
   <----------------------------- [0x01 ACK] -- Write verified & FPGA ready

2. Image / Spike Streaming (25 Timesteps):
   [0xBB Start Command] ----------------------> fifo_uart_controller (SEND_ACK)
   <----------------------------- [0x01 ACK] -- Ready for spikes
   For t = 0 to 24:
     [512 Bytes Spikes] ----------------------> FIFO (Unpacker -> Cascaded MAC)
     <--------------------------- [0x01 ACK] -- Timestep ACK (flow control)
   
3. Hardware Classification:
   - LIF Neuron integration & Rate decoding (master_fsm)
   - LED[0] (result_led) = 1 if Collision, 0 if Clear
   - LED[1] (done_led)   = 1 when inference complete
```

---

## 📁 Supported File Formats

### 1. Weights Files (`-w / --weights`)
- **Binary (`.bin`, `.dat`)**: Exactly 8192 bytes (4096 words $\times$ 16-bit signed, Big-Endian).
- **Text (`.txt`, `.csv`)**: 4096 numbers (one per line or space/comma separated) in decimal (`-250`, `1400`) or hex (`0x0A20`).

### 2. Image / Spike Files (`-i / --image`)
- **Raw Dataset PGM / PPM Images (`.pgm`, `.ppm`)**: Directly pass any raw image from the Zurich dataset (e.g. `DSCN2571_frame_181.pgm`, $324 \times 244$ or any arbitrary size). `snn_inference` automatically downsamples it using bilinear interpolation to $64 \times 64$, generates rate-coded spikes across 25 timesteps ($12,800$ bytes), and streams it to the FPGA over UART.
- **64x64 Netpbm Bitmaps (`.pbm`, `.pgm`)**: Netpbm binary/ASCII bitmap or grayscale image.
- **Spike Stream Binary (`.bin`)**: Pre-encoded 12,800 bytes ($25\text{ timesteps} \times 512\text{ bytes}$).
- **Single Frame Binary (`.bin`)**: 512 bytes (4096 bits) — automatically replicated across 25 timesteps.
- **Text / CSV (`.txt`)**: 4096 or 12,800 binary values (0 or 1).
