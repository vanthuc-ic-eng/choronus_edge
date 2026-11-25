// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/pkg/chronos_pkg.sv
// Desc   : Global architectural parameters, bit-exact numeric formats,
//          CSR address map, and micro-mode encodings.
// Target : FPGA (Xilinx Zynq-7000 @ >=100 MHz, 0 DSP48E1) & ASIC (Sky130)
// ============================================================================

`timescale 1ns / 1ps

/* verilator lint_off UNUSEDPARAM */
package chronos_pkg;

    // ------------------------------------------------------------------------
    // 1. Default Tiny-Mamba / S4D Architectural Geometry
    // ------------------------------------------------------------------------
    parameter int unsigned DEF_D_IN     = 16;   // Input feature dimension
    parameter int unsigned DEF_D_MODEL  = 32;   // Expanded SSM channel dimension D
    parameter int unsigned DEF_N_STATE  = 4;    // SSM state order N (matches 4-cycle overlap)
    parameter int unsigned DEF_CONV_K   = 4;    // Causal 1D Conv kernel width K
    parameter int unsigned DEF_NUM_PE   = 4;    // Engine A SIMD lanes (matches 32b 4xLog8 BRAM)
    parameter int unsigned DEF_NUM_CLS  = 8;    // Max output classes (5 AAMI ECG / up to 12 KWS)

    // ------------------------------------------------------------------------
    // 2. Bit-Exact Fixed-Point & Logarithmic Formats
    // ------------------------------------------------------------------------
    //   - Activation (INT8)      : signed  8-bit (Q4.4 for SiLU/Softplus, INT8 for MAC)
    //   - Weight (Log8)          : {sign[7], k[6:4], m[3:0]}, 8'h7F = Exact Zero
    //   - Log8 Unpacked (Q3.6)   : 9-bit unsigned {k[2:0], frac[5:0]}
    //   - State h[t] (INT16)     : signed 16-bit Q5.10 (range [-32.0, +31.999])
    //   - State Log (Q4.6)       : 10-bit unsigned {k[3:0], frac[5:0]}
    //   - Delta[t] (Q2.6)        : unsigned 8-bit strictly positive (from Softplus)
    //   - A_mag (Log8_u)         : unsigned 8-bit Q3.5 / Log2(|A_{d,n}|)
    parameter logic [7:0]  LOG8_ZERO_CODE   = 8'h7F;
    parameter logic [9:0]  MIN_LYAP_DECAY   = 10'd1;    // >= 2^(-1/64) spectral stability clamp
    parameter logic signed [15:0] STATE_MAX = 16'sd32767;
    parameter logic signed [15:0] STATE_MIN = -16'sd32767;

    // ------------------------------------------------------------------------
    // 3. Engine B Micro-Mode Encoding (Ultra-Clean 2-Mode Core)
    // ------------------------------------------------------------------------
    typedef enum logic [0:0] {
        SSM_MODE_RECURR = 1'b0,   // Log-ZOH h[t] = exp(dA)*h[t-1] + dB*x[t] & fused y += C*h[t]
        SSM_MODE_GATE   = 1'b1    // Element-wise log gate: out_d = y_d * SiLU(z_d)
    } ssm_submode_e;

    // ------------------------------------------------------------------------
    // 4. Non-Linear PWL Unit Function Select
    // ------------------------------------------------------------------------
    typedef enum logic [0:0] {
        NL_FUNC_SILU     = 1'b0,  // SiLU(z) = z * sigmoid(z)
        NL_FUNC_SOFTPLUS = 1'b1   // Softplus(u) = ln(1 + exp(u))
    } nl_func_e;

    // ------------------------------------------------------------------------
    // 5. Autonomous HWPE Scheduler FSM States
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        HWPE_IDLE          = 3'd0,
        HWPE_STEP_CHK      = 3'd1,   // Level-1 Coarse Input-Activity Silent-Frame Check
        HWPE_IN_PROJ       = 3'd2,   // Engine A: Linear_in (u -> x_raw, z) + Trainable Log8 Conv1D
        HWPE_BC_PROJ       = 3'd3,   // Engine A: Selective B[0..N-1], C[0..N-1] projections
        HWPE_FUSED_OVERLAP = 3'd4,   // ★ True 4+1 Concurrent Overlap: Engine A (Δ_g) || Engine B (SSM_{g-1})
        HWPE_OUT_PROJ      = 3'd5,   // Engine A: Final Classifier Linear Projection
        HWPE_ARGMAX        = 3'd6,   // 2-stage binary-tree Argmax + Anomaly IRQ evaluation
        HWPE_DONE          = 3'd7
    } hwpe_state_e;

    // ------------------------------------------------------------------------
    // 6. Memory-Mapped 32-bit CSR Register Offset Map (Base: 0x4000_0000)
    // ------------------------------------------------------------------------
    parameter logic [7:0] CSR_CTRL           = 8'h00; // [0]START [1]DONE [2]STREAM [3]STATE_RST [4]STEP_SKIP_EN [5]MODE_S6 [6]LYAP_EN [7]CHAN_SKIP_EN
    parameter logic [7:0] CSR_STATUS         = 8'h04; // [2:0]FSM [3]IRQ_PEND [4]RING_FULL [5]BUSY
    parameter logic [7:0] CSR_SSM_CFG        = 8'h08; // [7:0]D_MODEL [15:8]N_STATE [23:16]D_IN [31:24]NUM_CLS
    parameter logic [7:0] CSR_SEQ_LEN        = 8'h0C; // [15:0]L_SEQ [31:16]STEP_IDX
    parameter logic [7:0] CSR_SKIP_THRESH    = 8'h10; // [7:0]TAU_CHAN [15:8]TAU_STEP
    parameter logic [7:0] CSR_CLASS_OUT      = 8'h14; // [3:0]PRED_CLS [31:16]MAX_LOGIT
    parameter logic [7:0] CSR_IRQ_CTRL       = 8'h18; // [0]IRQ_EN [1]IRQ_CLR [15:8]ANOMALY_MASK
    // Hardware Performance, Concurrency & Power-Saving Telemetry Counters (Beyond-Q1)
    parameter logic [7:0] CSR_TOTAL_CYCLES   = 8'h20; // Total HWPE clock cycles elapsed
    parameter logic [7:0] CSR_ACTIVE_CYCLES  = 8'h24; // Cycles Engine B was active (un-gated)
    parameter logic [7:0] CSR_CHAN_SKIP_CNT  = 8'h28; // Level-2 Fine-grained channel skips
    parameter logic [7:0] CSR_STEP_SKIP_CNT  = 8'h2C; // Level-1 Coarse silent-step early-exits
    parameter logic [7:0] CSR_SRAM_SAVED_CNT = 8'h30; // Bank-1 Dual-Port SRAM R/W accesses avoided
    parameter logic [7:0] CSR_ENERGY_PROXY   = 8'h34; // PMU weighted dynamic energy proxy counter
    parameter logic [7:0] CSR_CPU_SLEEP_CYC  = 8'h38; // PMU RISC-V WFI clock-gated sleep cycles
    parameter logic [7:0] CSR_OVERLAP_CYC    = 8'h3C; // ★ True 4+1 Concurrent (EngA && EngB) cycles!

endpackage : chronos_pkg
/* verilator lint_on UNUSEDPARAM */
