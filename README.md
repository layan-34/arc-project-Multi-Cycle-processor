# ⚙️ Multi-Cycle Processor — Verilog HDL

> **Course:** ENCS4370 — Computer Architecture  
> **Semester:** Spring 2024/2025  
> **Author:** [layan-34](https://github.com/layan-34)  
> **Repository:** [github.com/layan-34/arc-project-Multi-Cycle-processor](https://github.com/layan-34/arc-project-Multi-Cycle-processor)

---

## 📌 Overview

A complete **Multi-Cycle Processor** designed and implemented in **Verilog HDL**, featuring a custom 16-bit ISA with 32-bit data paths. The processor executes instructions across multiple clock cycles (Fetch → Decode → Execute → Memory → Writeback) with full control unit state machine, ALU, register file, and memory subsystem.

| Component | Description |
|-----------|-------------|
| **Control Unit** | FSM-based multi-cycle controller with 6 states |
| **Datapath** | 32-bit datapath with pipeline registers and multiplexers |
| **ALU** | Supports OR, ADD, SUB, CMP with flag generation |
| **Register File** | 16 × 32-bit general-purpose registers (flip-flop based) |
| **Memory** | Separate instruction memory (1024 words) and data memory |

---

## ✨ Features

- 🔄 **Multi-Cycle Execution** — 5-stage FSM: FETCH → DECODE → EXECUTE → MEMORY → WRITEBACK
- 🧮 **Custom 16-Instruction ISA** — Supports R-type, I-type, memory, branch, and jump instructions
- 📐 **32-bit Data Path** — Full 32-bit ALU, registers, and memory bus
- 🔀 **Double-Word Operations** — LDW/SDW instructions for loading/storing 64-bit data across two cycles
- ⚠️ **Exception Handling** — Detects odd register addresses for LDW/SDW and invalid opcodes
- 🧪 **Comprehensive Testbenches** — Individual testbenches for every module (ALU, Control Unit, CPU, Datapath, etc.)
- 📊 **Debug Outputs** — Full observability of internal state, ALU results, register values, and memory signals

---

## 🏗️ Instruction Set Architecture (ISA)

### R-Type Instructions
| Opcode | Mnemonic | Operation | Description |
|--------|----------|-----------|-------------|
| `000000` | OR | Rd = Rs \| Rt | Bitwise OR |
| `000001` | ADD | Rd = Rs + Rt | Addition |
| `000010` | SUB | Rd = Rs − Rt | Subtraction |
| `000011` | CMP | Rd = cmp(Rs, Rt) | Compare (sets flags) |

### I-Type Instructions
| Opcode | Mnemonic | Operation | Description |
|--------|----------|-----------|-------------|
| `000100` | ORI | Rd = Rs \| imm | OR Immediate (zero-extended) |
| `000101` | ADDI | Rd = Rs + imm | Add Immediate (sign-extended) |

### Memory Instructions
| Opcode | Mnemonic | Operation | Description |
|--------|----------|-----------|-------------|
| `000110` | LW | Rd = MEM[Rs + imm] | Load Word |
| `000111` | SW | MEM[Rs + imm] = Rd | Store Word |
| `001000` | LDW | Rd, Rd+1 = MEM[Rs + imm] | Load Double Word (2 cycles) |
| `001001` | SDW | MEM[Rs + imm] = Rd, Rd+1 | Store Double Word (2 cycles) |

### Branch & Jump Instructions
| Opcode | Mnemonic | Operation | Description |
|--------|----------|-----------|-------------|
| `001010` | BZ | if (zero) PC += imm | Branch if Zero |
| `001011` | BGZ | if (positive) PC += imm | Branch if Greater than Zero |
| `001100` | BLZ | if (negative) PC += imm | Branch if Less than Zero |
| `001101` | JR | PC = Rs | Jump Register |
| `001110` | J | PC = target | Unconditional Jump |
| `001111` | CALL | R14 = PC+1; PC = target | Function Call |

### Instruction Format (32-bit)
```
┌────────┬──────┬──────┬──────┬────────────────┐
│ Opcode │  Rd  │  Rs  │  Rt  │   Immediate    │
│ [31:26]│[25:22]│[21:18]│[17:14]│   [13:0]     │
│ 6 bits │4 bits│4 bits│4 bits│   14 bits      │
└────────┴──────┴──────┴──────┴────────────────┘
```

---

## 🚀 How to Run

### Prerequisites

- **Xilinx Vivado**, **ModelSim**, [Icarus Verilog](http://iverilog.icarus.com/), or any Verilog simulator
- Optional: **GTKWave** for waveform viewing

### Steps

1. **Clone the repository:**
   ```bash
   git clone https://github.com/layan-34/arc-project-Multi-Cycle-processor.git
   cd arc-project-Multi-Cycle-processor
   ```

2. **Open in your simulator:**
   - **Vivado:** Create a new project and add all `.v` files. Open `arcticture_project.aws` for the workspace.
   - **ModelSim:** Compile all `.v` files and run `test.v` or individual testbenches.
   - **Icarus Verilog:**
     ```bash
     iverilog -o sim *.v
     vvp sim
     ```

3. **Run testbenches** for individual modules:
   ```bash
   # Example: Run ALU testbench
   iverilog -o alu_sim ALU.v
   vvp alu_sim
   ```

4. **View waveforms** (if VCD files are generated):
   ```bash
   gtkwave Alu.vcd
   ```

---

## 📁 Project Structure

```
arc-project-Multi-Cycle-processor/
├── computer.v                                    # Top-level module (CPU + memories)
├── cpu.v                                         # CPU core (datapath + control unit)
├── control_unit.v                                # FSM-based multi-cycle control unit
├── datapath.v                                    # Datapath with pipeline registers
├── ALU.v                                         # 32-bit ALU with flag generation
├── registerFile.v                                # 16 × 32-bit register file
├── data_memory.v                                 # Data memory with LDW/SDW support
├── instructionMemory.v                           # 1024-word instruction memory
├── MUX.v                                         # 4-to-1 multiplexers (4-bit & 32-bit)
├── EXT.v                                         # Sign/Zero extender
├── flipflop.v                                    # 32-bit flip-flop with write enable
├── pc.v                                          # Program counter module
├── processor.v                                   # Alternative processor wrapper
├── test.v                                        # Top-level integration testbench
├── truth table.xlsx                              # Control signal truth table
├── arcticture_project.aws                        # Vivado workspace file
├── Project-2-Spring-2024-2025.pdf                # Project specification
├── Project2Report.pdf                            # Final project report
└── README.md                                     # This file
```

---

## 🧠 Multi-Cycle FSM

The processor uses a **Finite State Machine (FSM)** to control instruction execution across multiple clock cycles:

```
┌─────────┐     ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌───────────┐
│  FETCH  │────▶│ DECODE  │────▶│ EXECUTE │────▶│ MEMORY  │────▶│ WRITEBACK │
│  (000)  │     │  (001)  │     │  (010)  │     │  (011)  │     │   (100)   │
└─────────┘     └────┬────┘     └────┬────┘     └────┬────┘     └─────┬─────┘
     ▲               │               │               │               │
     │               ▼               │               ▼               │
     │         ┌───────────┐         │         ┌──────────┐          │
     │         │ EXCEPTION │         └────────▶│ 2nd Cycle│          │
     │         │   (101)   │         (branch/  │ (LDW/SDW)│          │
     │         └───────────┘          jump)    └──────────┘          │
     └──────────────────────────────────────────────────────────────┘
```

### Cycle Count per Instruction

| Instruction Type | Cycles | Stages |
|------------------|--------|--------|
| ALU (R-type) | 4 | Fetch → Decode → Execute → Writeback |
| ALU (I-type) | 4 | Fetch → Decode → Execute → Writeback |
| Load Word (LW) | 5 | Fetch → Decode → Execute → Memory → Writeback |
| Store Word (SW) | 4 | Fetch → Decode → Execute → Memory |
| Load Double Word (LDW) | 6 | Fetch → Decode → Execute → Memory × 2 → Writeback |
| Store Double Word (SDW) | 5 | Fetch → Decode → Execute → Memory × 2 |
| Branch (BZ/BGZ/BLZ) | 3 | Fetch → Decode → Execute |
| Jump (J/JR/CALL) | 3 | Fetch → Decode → Execute |

---

## 🛠️ Technical Highlights

- **FSM Control Unit** — 6-state machine generating 15+ control signals per cycle, with proper state transitions for all 16 instructions
- **Pipeline Registers** — IF/ID, ID/EX, EX/MEM, MEM/WB registers isolate each pipeline stage
- **Double-Word Support** — LDW/SDW instructions use an internal cycle counter to perform two memory operations across consecutive addresses
- **Exception Detection** — Hardware checks for odd register addresses on LDW/SDW and invalid opcodes (opcode > 0x0F)
- **Register File** — 16 general-purpose 32-bit registers implemented with flip-flops; R15 is hardwired to the PC
- **ALU Flags** — Zero, Negative, and Positive flags drive conditional branching
- **Sign/Zero Extension** — Configurable 14-bit to 32-bit extender for immediate operands
- **Modular Design** — Each component has its own testbench for independent verification

---

## 📄 Exception Handling

| Exception | Condition | Behavior |
|-----------|-----------|----------|
| **LDW Odd Register** | `Rd[0] == 1` | Traps to EXCEPTION state |
| **SDW Odd Register** | `Rs[0] == 1` | Traps to EXCEPTION state |
| **Invalid Opcode** | `opcode > 0x0F` | Traps to EXCEPTION state |

When an exception is detected during the DECODE stage, the processor enters the `EXCEPTION_STATE` and remains there until a hardware reset.

---

## 🔗 Links

| Resource | URL |
|----------|-----|
| 📦 **This Repository** | [github.com/layan-34/arc-project-Multi-Cycle-processor](https://github.com/layan-34/arc-project-Multi-Cycle-processor) |
| 📝 **Assembly Project** | [github.com/layan-34/arc-project-assymply-program](https://github.com/layan-34/arc-project-assymply-program) |
| 👤 **GitHub Profile** | [github.com/layan-34](https://github.com/layan-34) |
| 📖 **Verilog Reference** | [Verilog HDL (Wikipedia)](https://en.wikipedia.org/wiki/Verilog) |
| 🔧 **Icarus Verilog** | [iverilog.icarus.com](http://iverilog.icarus.com/) |

---

## 📝 License

This project was developed as a university assignment for ENCS4370 — Computer Architecture.

---

<p align="center">
  Made with 💻 in Verilog HDL by <a href="https://github.com/layan-34">layan-34</a>
</p>