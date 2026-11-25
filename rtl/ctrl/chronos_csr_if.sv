// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/ctrl/chronos_csr_if.sv
// Desc   : Memory-Mapped 32-bit CSR File + Hardware Power/Skip Telemetry.
//          Drop-in compatible with PULP RISC-V RV32I peripheral bus.
// ============================================================================

`timescale 1ns / 1ps

module chronos_csr_if (
    input  logic        clk_i,
    input  logic        rst_ni,

    // 32-bit Memory-Mapped CSR Bus
    input  logic        csr_req_i,
    input  logic        csr_we_i,
    input  logic [7:0]  csr_addr_i,
    input  logic [31:0] csr_wdata_i,
    output logic [31:0] csr_rdata_o,

    // Control Outputs to HWPE Scheduler & Engines
    output logic        start_pulse_o,
    output logic        stream_en_o,
    output logic        state_rst_pulse_o,
    output logic        step_skip_en_o,
    output logic        chan_skip_en_o,
    output logic        mode_s6_o,          // 0 = Static S4D, 1 = Selective Mamba
    output logic        lyap_guard_en_o,    // 1 = Enforce spectral stability clamp
    output logic [7:0]  cfg_d_model_o,
    output logic [7:0]  cfg_n_state_o,
    output logic [7:0]  cfg_d_in_o,
    output logic [7:0]  cfg_num_cls_o,
    output logic [15:0] cfg_seq_len_o,
    output logic [7:0]  cfg_tau_chan_o,
    output logic [7:0]  cfg_tau_step_o,
    output logic        irq_en_o,
    output logic [7:0]  anomaly_mask_o,
    output logic        irq_o,

    // Status & Telemetry Inputs from HWPE
    input  logic [2:0]  fsm_state_i,
    input  logic        busy_i,
    input  logic        done_pulse_i,
    input  logic        ring_full_i,
    input  logic [15:0] step_idx_i,
    input  logic [3:0]  pred_class_i,
    input  logic signed [15:0] max_logit_i,
    input  logic        anomaly_irq_pulse_i,
    input  logic [31:0] total_cyc_i,
    input  logic [31:0] active_cyc_i,
    input  logic [31:0] chan_skip_cnt_i,
    input  logic [31:0] step_skip_cnt_i,
    input  logic [31:0] sram_saved_cnt_i,
    input  logic [31:0] energy_proxy_i,
    input  logic [31:0] cpu_sleep_cyc_i,
    input  logic [31:0] overlap_cyc_i
);

    logic        done_sticky_q;
    logic        irq_pending_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            start_pulse_o     <= 1'b0;
            state_rst_pulse_o <= 1'b0;
            stream_en_o       <= 1'b0;
            step_skip_en_o    <= 1'b0;
            chan_skip_en_o    <= 1'b0;
            mode_s6_o         <= 1'b1;      // Default: Selective Mamba
            lyap_guard_en_o   <= 1'b1;      // Default: Lyapunov Guard ON
            cfg_d_model_o     <= 8'd16;
            cfg_n_state_o     <= 8'd4;
            cfg_d_in_o        <= 8'd8;
            cfg_num_cls_o     <= 8'd5;      // Default: 5 AAMI ECG classes
            cfg_seq_len_o     <= 16'd1;
            cfg_tau_chan_o    <= 8'd0;
            cfg_tau_step_o    <= 8'd0;
            irq_en_o          <= 1'b0;
            anomaly_mask_o    <= 8'hFE;     // Default: classes 1..7 trigger arrhythmia wakeup
            done_sticky_q     <= 1'b0;
            irq_pending_q     <= 1'b0;
        end else begin
            // Self-clearing command pulses
            start_pulse_o     <= 1'b0;
            state_rst_pulse_o <= 1'b0;

            if (done_pulse_i) begin
                done_sticky_q <= 1'b1;
            end
            if (anomaly_irq_pulse_i) begin
                irq_pending_q <= 1'b1;
            end

            if (csr_req_i && csr_we_i) begin
                case (csr_addr_i)
                    chronos_pkg::CSR_CTRL: begin
                        if (csr_wdata_i[0]) begin
                            start_pulse_o <= 1'b1;
                            done_sticky_q <= 1'b0;
                        end
                        if (csr_wdata_i[1]) begin
                            done_sticky_q <= 1'b0; // W1C
                        end
                        stream_en_o       <= csr_wdata_i[2];
                        state_rst_pulse_o <= csr_wdata_i[3];
                        step_skip_en_o    <= csr_wdata_i[4];
                        mode_s6_o         <= csr_wdata_i[5];
                        lyap_guard_en_o   <= csr_wdata_i[6];
                        chan_skip_en_o    <= csr_wdata_i[7];
                    end
                    chronos_pkg::CSR_SSM_CFG: begin
                        cfg_d_model_o <= csr_wdata_i[7:0];
                        cfg_n_state_o <= csr_wdata_i[15:8];
                        cfg_d_in_o    <= csr_wdata_i[23:16];
                        cfg_num_cls_o <= csr_wdata_i[31:24];
                    end
                    chronos_pkg::CSR_SEQ_LEN: begin
                        cfg_seq_len_o <= csr_wdata_i[15:0];
                    end
                    chronos_pkg::CSR_SKIP_THRESH: begin
                        cfg_tau_chan_o <= csr_wdata_i[7:0];
                        cfg_tau_step_o <= csr_wdata_i[15:8];
                    end
                    chronos_pkg::CSR_IRQ_CTRL: begin
                        irq_en_o       <= csr_wdata_i[0];
                        if (csr_wdata_i[1]) irq_pending_q <= 1'b0; // IRQ_CLR
                        anomaly_mask_o <= csr_wdata_i[15:8];
                    end
                    default: ;
                endcase
            end
        end
    end

    assign irq_o = irq_pending_q && irq_en_o;

    // Combinational CSR Readback
    always_comb begin
        case (csr_addr_i)
            chronos_pkg::CSR_CTRL:
                csr_rdata_o = {24'd0, chan_skip_en_o, lyap_guard_en_o, mode_s6_o,
                               step_skip_en_o, 1'b0, stream_en_o, done_sticky_q, 1'b0};
            chronos_pkg::CSR_STATUS:
                csr_rdata_o = {26'd0, busy_i, ring_full_i, irq_pending_q, fsm_state_i};
            chronos_pkg::CSR_SSM_CFG:
                csr_rdata_o = {cfg_num_cls_o, cfg_d_in_o, cfg_n_state_o, cfg_d_model_o};
            chronos_pkg::CSR_SEQ_LEN:
                csr_rdata_o = {step_idx_i, cfg_seq_len_o};
            chronos_pkg::CSR_SKIP_THRESH:
                csr_rdata_o = {16'd0, cfg_tau_step_o, cfg_tau_chan_o};
            chronos_pkg::CSR_CLASS_OUT:
                csr_rdata_o = {max_logit_i, 12'd0, pred_class_i};
            chronos_pkg::CSR_IRQ_CTRL:
                csr_rdata_o = {16'd0, anomaly_mask_o, 6'd0, irq_pending_q, irq_en_o};
            chronos_pkg::CSR_TOTAL_CYCLES:   csr_rdata_o = total_cyc_i;
            chronos_pkg::CSR_ACTIVE_CYCLES:  csr_rdata_o = active_cyc_i;
            chronos_pkg::CSR_CHAN_SKIP_CNT:  csr_rdata_o = chan_skip_cnt_i;
            chronos_pkg::CSR_STEP_SKIP_CNT:  csr_rdata_o = step_skip_cnt_i;
            chronos_pkg::CSR_SRAM_SAVED_CNT: csr_rdata_o = sram_saved_cnt_i;
            chronos_pkg::CSR_ENERGY_PROXY:   csr_rdata_o = energy_proxy_i;
            chronos_pkg::CSR_CPU_SLEEP_CYC:  csr_rdata_o = cpu_sleep_cyc_i;
            chronos_pkg::CSR_OVERLAP_CYC:    csr_rdata_o = overlap_cyc_i;
            default:                         csr_rdata_o = 32'd0;
        endcase
    end

endmodule : chronos_csr_if
