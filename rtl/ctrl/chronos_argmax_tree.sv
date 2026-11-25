// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : rtl/ctrl/chronos_argmax_tree.sv
// Desc   : 2-Stage Pipelined Binary-Tree Argmax (up to 8 classes) +
//          Event-Driven Anomaly IRQ Generator (wakes host RISC-V only when
//          predicted class matches programmable anomaly bitmask).
// ============================================================================

`timescale 1ns / 1ps

module chronos_argmax_tree (
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               valid_i,
    input  logic        [3:0]  num_cls_i,       // Active classes (1..8)

    // Up to 8 Signed 16-bit Class Logits
    input  logic signed [15:0] logit0_i,
    input  logic signed [15:0] logit1_i,
    input  logic signed [15:0] logit2_i,
    input  logic signed [15:0] logit3_i,
    input  logic signed [15:0] logit4_i,
    input  logic signed [15:0] logit5_i,
    input  logic signed [15:0] logit6_i,
    input  logic signed [15:0] logit7_i,

    // Anomaly Wakeup IRQ Mask
    input  logic               irq_en_i,
    input  logic        [7:0]  anomaly_mask_i,

    output logic               valid_o,
    output logic        [3:0]  pred_class_o,
    output logic signed [15:0] max_logit_o,
    output logic               anomaly_irq_pulse_o
);

    localparam logic signed [15:0] NEG_INF = -16'sd32768;

    // Mask out inactive classes beyond num_cls_i
    logic signed [15:0] v [0:7];
    always_comb begin
        v[0] = (4'd0 < num_cls_i) ? logit0_i : NEG_INF;
        v[1] = (4'd1 < num_cls_i) ? logit1_i : NEG_INF;
        v[2] = (4'd2 < num_cls_i) ? logit2_i : NEG_INF;
        v[3] = (4'd3 < num_cls_i) ? logit3_i : NEG_INF;
        v[4] = (4'd4 < num_cls_i) ? logit4_i : NEG_INF;
        v[5] = (4'd5 < num_cls_i) ? logit5_i : NEG_INF;
        v[6] = (4'd6 < num_cls_i) ? logit6_i : NEG_INF;
        v[7] = (4'd7 < num_cls_i) ? logit7_i : NEG_INF;
    end

    // Stage 1: 4-way pairwise comparison (8 -> 4)
    logic signed [15:0] s1_val [0:3];
    logic        [2:0]  s1_idx [0:3];
    logic               s1_valid_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            s1_valid_q <= 1'b0;
            for (int i = 0; i < 4; i++) begin
                s1_val[i] <= NEG_INF;
                s1_idx[i] <= 3'd0;
            end
        end else begin
            s1_valid_q <= valid_i;
            for (int i = 0; i < 4; i++) begin
                if (v[2*i] >= v[2*i + 1]) begin
                    s1_val[i] <= v[2*i];
                    s1_idx[i] <= {i[1:0], 1'b0};
                end else begin
                    s1_val[i] <= v[2*i + 1];
                    s1_idx[i] <= {i[1:0], 1'b1};
                end
            end
        end
    end

    // Stage 2: Final 4 -> 2 -> 1 comparison
    logic signed [15:0] l_val, r_val, final_val;
    logic        [2:0]  l_idx, r_idx, final_idx;

    always_comb begin
        if (s1_val[0] >= s1_val[1]) begin l_val = s1_val[0]; l_idx = s1_idx[0]; end
        else                        begin l_val = s1_val[1]; l_idx = s1_idx[1]; end

        if (s1_val[2] >= s1_val[3]) begin r_val = s1_val[2]; r_idx = s1_idx[2]; end
        else                        begin r_val = s1_val[3]; r_idx = s1_idx[3]; end

        if (l_val >= r_val) begin final_val = l_val; final_idx = l_idx; end
        else                begin final_val = r_val; final_idx = r_idx; end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_o             <= 1'b0;
            pred_class_o        <= 4'd0;
            max_logit_o         <= 16'sd0;
            anomaly_irq_pulse_o <= 1'b0;
        end else begin
            valid_o <= s1_valid_q;
            if (s1_valid_q) begin
                pred_class_o        <= {1'b0, final_idx};
                max_logit_o         <= final_val;
                anomaly_irq_pulse_o <= irq_en_i && anomaly_mask_i[final_idx];
            end else begin
                anomaly_irq_pulse_o <= 1'b0;
            end
        end
    end

endmodule : chronos_argmax_tree
