# SNN Accelerator with Raw Ethernet Interface (100 Mbps MII)

High-performance Spiking Neural Network (SNN) hardware accelerator for obstacle detection and collision prediction on the Digilent Arty A7-100T FPGA board, communicating via Raw Ethernet (IEEE 802.3 / EtherType `0x88B5`).

---

## 📁 Directory Structure

```
SNN_accelerator_eth/
├── rtl/                        # FPGA Verilog RTL Source Files
│   ├── snn_eth_top.v           # Top-level module
│   ├── fifo_eth_controller.v   # MII 100Mbps Ethernet + Spike FIFO + BRAM loader
│   ├── master_fsm.v            # Master FSM coordinating pipeline & timesteps
│   ├── cascaded_adder.v        # High-throughput pipelined MAC accumulator
│   ├── lif_neuron.v            # Leaky Integrate-and-Fire neuron model
│   ├── spike_counter.v         # Rate-coding spike accumulator & threshold decoder
│   ├── spike_unpacker.v        # Bit-serial spike unpacker (MSB-first)
│   └── weight_bram.v           # 8KB True Dual-Port Block RAM
├── tb/                         # Simulation Testbenches & Scripts
│   ├── tb_snn_eth_top.v        # Full end-to-end MII Ethernet simulation
│   └── run_sim.sh              # Vivado xsim batch simulation runner
├── constraints/                # Pin & Timing Constraints
│   └── arty_a7_100t_eth.xdc    # Arty A7-100T PHY MII & 100MHz clock constraints
├── host/                       # Linux C Host Driver & CLI Application
│   ├── main.c                  # CLI interface (single image, batch, ping, sim)
│   ├── eth_comm.c / eth_comm.h # Raw socket AF_PACKET driver
│   ├── snn_protocol.h          # Ethernet protocol commands & hardware constants
│   ├── snn_sim.c / snn_sim.h   # Software reference SNN model
│   ├── file_io.c / file_io.h   # Image / dataset parsers
│   └── Makefile                # Host application build script
├── scripts/                    # Vivado Automation Scripts
│   ├── build_bitstream.tcl     # Synthesis, implementation & bitstream flow
│   └── program_fpga.tcl        # Hardware Manager flashing script
└── build/                      # Bitstream (snn_eth_top.bit) & reports
```

---

## 🚀 Quick Start Guide


### 1. Flash Bitstream to Arty A7-100T FPGA

### 2. Build Host Application
```bash
cd host
make
```

### 3. Ping FPGA over Ethernet
Test physical link and FPGA state:
```bash
sudo ./snn_inference_eth --ifname eno1 --ping
```

### 4. Run SNN Inference over Ethernet
- **Single Image Inference:**
  ```bash
  sudo ./snn_inference_eth --ifname eno1 --weights ../data/weights.bin --image ../data/sample_collision_img.bin --ascii
  ```

- **Batch Directory Inference:**
  ```bash
  sudo ./snn_inference_eth --ifname eno1 --weights ../data/weights.bin --dir ../data/ --csv results.csv
  ```

- **Offline Simulation Mode (No Root/FPGA required):**
  ```bash
  ./snn_inference_eth --sim --weights ../data/weights.bin --image ../data/sample_collision_img.bin
  ```

---

## 📡 Raw Ethernet Protocol Specification

- **EtherType:** `0x88B5` (IEEE 802 Local Experimental)
- **FPGA MAC:** `00:18:3E:04:C5:52` (Default / configurable via `FPGA_MAC` parameter or `-m` CLI option)

> [!IMPORTANT]
> **FPGA Hardware MAC Address:**
> Replace the MAC address with the hardware MAC address of your specific FPGA board (printed on the sticker on the Digilent Arty A7 board):
> 1. **In RTL:** Update the `FPGA_MAC` parameter in [`rtl/snn_eth_top.v`](file:///home/sasi/wrk/SNN_accelerator_eth/rtl/snn_eth_top.v) and [`rtl/fifo_eth_controller.v`](file:///home/sasi/wrk/SNN_accelerator_eth/rtl/fifo_eth_controller.v) before generating the bitstream.
> 2. **In Host CLI:** Either update `DEFAULT_FPGA_MAC_STR` in [`host/snn_protocol.h`](file:///home/sasi/wrk/SNN_accelerator_eth/host/snn_protocol.h) or supply your board's MAC address at runtime using the `-m` / `--mac` flag (e.g., `-m 00:18:3E:04:C5:52`).

### Command Frames (Host -> FPGA)
| Command | Name | Payload Structure | Description |
|---|---|---|---|
| `0xAA` | `WRITE_WEIGHTS` | `[0xAA] [blk: 0..7] [1024 bytes weights]` | Writes 512 16-bit signed weights to BRAM (8 blocks total) |
| `0xBB` | `WRITE_SPIKES` | `[0xBB] [t: 0..24] [512 bytes spikes]` | Writes 4096 spike bits into FIFO for timestep `t` |
| `0xCC` | `INFER_REQ` | `[0xCC]` | Queries last inference result |
| `0xDD` | `PING` | `[0xDD]` | Queries FPGA alive status |

### Response Frames (FPGA -> Host)
| Status | Name | Payload Structure | Description |
|---|---|---|---|
| `0x01` | `ACK` | `[0x01] [sub_idx] [param2]` | Acknowledges weight block or spike timestep |
| `0x02` | `RESULT` | `[0x02] [class: 0/1] [param2]` | Pushes classification result (0=Clear, 1=Collision) |
| `0x03` | `PONG` | `[0x03] [weights_loaded] [fifo_empty]` | Pong response to ping query |
