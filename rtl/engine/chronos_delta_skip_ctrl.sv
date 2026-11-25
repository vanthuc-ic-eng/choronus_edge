// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/engine/chronos_delta_skip_ctrl.sv
// Desc   : ★ Q1 CORE INNOVATION 4 — Dual-Granularity Selective Δ-Skipping ★
//          Hardware Power-Management & Telemetry Controller:
//            1) Fine-Grained Channel Skip: If Δ_d[t] < tau_chan -> gates
//               Engine B clock (ICG) AND suppresses Bank-1 SRAM reads/writes
//               for all N states of channel d!
//            2) Coarse-Grained Step Skip: Accumulates mean(Δ[t]) across all D
//               channels; flags step-level sparsity when mean(Δ) < tau_step.
//            3) Exact Hardware Telemetry Counters for IEEE Q1 PPA reporting.
// ============================================================================

`timescale 1ns / 1ps

module chronos_delta_skip_ctrl (
    input  logic        clk_i,
    input  logic        rst_ni,
    input  logic        cnt_clr_i,        // Clear telemetry counters on new run

    // Configuration from CSR
    input  logic        step_skip_en_i,   // Enable coarse step-skip detection
    input  logic        chan_skip_en_i,   // Enable fine-grained channel-skip
    input  logic [7:0]  tau_chan_i,       // Per-channel Δ threshold (Q2.6)
    input  logic [7:0]  tau_step_i,       // Per-step mean Δ threshold (Q2.6)
    input  logic [7:0]  n_state_i,        // State order N (for SRAM access savings)

    // Real-time Δ_d[t] Evaluation & Level-1 Coarse Step Early-Exit Interface
    input  logic        step_early_exit_i,// Pulse when Level-1 Activity Gate skips whole timestep t
    input  logic        delta_eval_i,     // Pulse when a new channel Δ_d[t] is ready
    input  logic        step_first_ch_i,  // High on d=0 of a timestep
    input  logic        step_last_ch_i,   // High on d=D-1 of a timestep
    input  logic [7:0]  delta_d_i,        // Current channel Δ_d[t] (Q2.6)
    input  logic [7:0]  d_model_i,        // Active channel count D (power of 2: 16/32/64)

    // Active compute status (for clock-gating telemetry)
    input  logic        ssm_busy_i,
    input  logic        ssm_active_i,     // High when Engine B is actually clocking

    // Immediate Gating Decisions
    output logic        chan_skip_o,      // 1 = Skip Engine B & Bank-1 R/W for this channel
    output logic        step_skip_o,      // 1 = Current timestep classified as static/silent

    // Telemetry Counters -> CSR Readback
    output logic [31:0] total_cyc_o,
    output logic [31:0] active_cyc_o,
    output logic [31:0] chan_skip_cnt_o,
    output logic [31:0] step_skip_cnt_o,
    output logic [31:0] sram_saved_cnt_o  // Number of Bank-1 R+W accesses eliminated (2*N per skip)
);

    // Combinational Fine-Grained Channel Skip Decision
    assign chan_skip_o = chan_skip_en_i && (delta_d_i < tau_chan_i);

    // Step-Level Mean Δ Accumulator (Zero-DSP power-of-two shift division)
    logic [15:0] delta_sum_q;
    logic [15:0] delta_sum_next;
    logic [7:0]  delta_mean;

    always_comb begin
        if (step_first_ch_i)
            delta_sum_next = {8'd0, delta_d_i};
        else
            delta_sum_next = delta_sum_q + {8'd0, delta_d_i};

        // Fast shift-divide for D in {8, 16, 32, 64}
        case (d_model_i)
            8'd8   : delta_mean = delta_sum_next[10:3];
            8'd16  : delta_mean = delta_sum_next[11:4];
            8'd32  : delta_mean = delta_sum_next[12:5];
            8'd64  : delta_mean = delta_sum_next[13:6];
            default: delta_mean = delta_sum_next[12:5];
        endcase
    end

    // Compute 2*D*N (zero-DSP shift) for Level-1 full-step Bank-1 SRAM access savings
    logic [15:0] full_step_sram_rw;
    always_comb begin
        case (n_state_i)
            8'd2   : full_step_sram_rw = {6'd0, d_model_i, 2'b00}; // 4*D
            8'd4   : full_step_sram_rw = {5'd0, d_model_i, 3'b000}; // 8*D
            8'd8   : full_step_sram_rw = {4'd0, d_model_i, 4'b0000}; // 16*D
            default: full_step_sram_rw = {5'd0, d_model_i, 3'b000};
        endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            delta_sum_q      <= 16'd0;
            step_skip_o      <= 1'b0;
            total_cyc_o      <= 32'd0;
            active_cyc_o     <= 32'd0;
            chan_skip_cnt_o  <= 32'd0;
            step_skip_cnt_o  <= 32'd0;
            sram_saved_cnt_o <= 32'd0;
        end else if (cnt_clr_i) begin
            delta_sum_q      <= 16'd0;
            step_skip_o      <= 1'b0;
            total_cyc_o      <= 32'd0;
            active_cyc_o     <= 32'd0;
            chan_skip_cnt_o  <= 32'd0;
            step_skip_cnt_o  <= 32'd0;
            sram_saved_cnt_o <= 32'd0;
        end else begin
            // Performance counters
            if (ssm_busy_i) begin
                total_cyc_o <= total_cyc_o + 32'd1;
            end
            if (ssm_active_i) begin
                active_cyc_o <= active_cyc_o + 32'd1;
            end

            // Level-1 Coarse Input-Domain Silent-Frame Early Exit!
            if (step_early_exit_i) begin
                step_skip_o      <= 1'b1;
                step_skip_cnt_o  <= step_skip_cnt_o + 32'd1;
                sram_saved_cnt_o <= sram_saved_cnt_o + {16'd0, full_step_sram_rw};
            end

            // Level-2 Fine-Grained Channel Delta Evaluation
            if (delta_eval_i) begin
                delta_sum_q <= delta_sum_next;

                if (chan_skip_o) begin
                    chan_skip_cnt_o  <= chan_skip_cnt_o + 32'd1;
                    // Each skipped channel saves N reads + N writes = 2*N Bank-1 SRAM accesses!
                    sram_saved_cnt_o <= sram_saved_cnt_o + {23'd0, n_state_i, 1'b0};
                end

                if (step_last_ch_i) begin
                    if (step_skip_en_i && (delta_mean < tau_step_i)) begin
                        step_skip_o     <= 1'b1;
                        step_skip_cnt_o <= step_skip_cnt_o + 32'd1;
                    end else begin
                        step_skip_o     <= 1'b0;
                    end
                end
            end
        end
    end

endmodule : chronos_delta_skip_ctrl
