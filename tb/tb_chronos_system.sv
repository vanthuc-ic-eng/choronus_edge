// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Hardware Engine
// File   : tb/tb_chronos_system.sv
// Desc   : Full-System SoC-Level HWPE Verification & Q1 Ablation Benchmark!
//          Tests chronos_ssm_hwpe_top end-to-end across 3 Q1 configurations:
//            - Run 1 (S4D Static Baseline)    : mode_s6=0, skip=0
//            - Run 2 (Selective Mamba Full)   : mode_s6=1, skip=0 + Anomaly IRQ
//            - Run 3 (Mamba + Dual Δ-Skipping): mode_s6=1, chan_skip=1, step_skip=1
//          Reports exact cycle counts, Engine B active duty ratio, channel
//          skips, and Bank-1 SRAM accesses saved via CSR telemetry!
// ============================================================================

`timescale 1ns / 1ps

module tb_chronos_system;

    import chronos_pkg::*;

    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk; // 100 MHz

    // CSR Interface
    logic        csr_req, csr_we;
    logic [7:0]  csr_addr;
    logic [31:0] csr_wdata, csr_rdata;
    logic        irq_o;

    // Streaming Input Port
    logic               stream_we, ring_full;
    logic signed [7:0]  stream_data;

    // Memory Load Ports
    logic         w_load_we, bias_load_we, a_load_we, d_load_we;
    logic [10:0]  w_load_addr;
    logic [31:0]  w_load_data;
    logic [6:0]   bias_load_addr;
    logic [127:0] bias_load_data;
    logic [8:0]   a_load_addr;
    logic [9:0]   a_load_data;
    logic [5:0]   d_load_addr;
    logic [15:0]  d_load_data;

    logic         busy_o, done_o;
    logic [3:0]   pred_class_o;
    logic         pmu_cnt_clr, pmu_eng_a_act, pmu_eng_b_act, pmu_sram_act;
    logic [31:0]  overlap_cyc_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)          overlap_cyc_q <= 32'd0;
        else if (pmu_cnt_clr) overlap_cyc_q <= 32'd0;
        else if (busy_o && pmu_eng_a_act && pmu_eng_b_act)
            overlap_cyc_q <= overlap_cyc_q + 32'd1;
    end

    chronos_ssm_hwpe_top dut (
        .clk_i                ( clk            ),
        .rst_ni               ( rst_n          ),
        .test_en_i            ( 1'b0           ),
        .csr_req_i            ( csr_req        ),
        .csr_we_i             ( csr_we         ),
        .csr_addr_i           ( csr_addr       ),
        .csr_wdata_i          ( csr_wdata      ),
        .csr_rdata_o          ( csr_rdata      ),
        .irq_o                ( irq_o          ),
        .stream_we_i          ( stream_we      ),
        .stream_data_i        ( stream_data    ),
        .ring_full_o          ( ring_full      ),
        .w_load_we_i          ( w_load_we      ),
        .w_load_addr_i        ( w_load_addr    ),
        .w_load_data_i        ( w_load_data    ),
        .bias_load_we_i       ( bias_load_we   ),
        .bias_load_addr_i     ( bias_load_addr ),
        .bias_load_data_i     ( bias_load_data ),
        .a_load_we_i          ( a_load_we      ),
        .a_load_addr_i        ( a_load_addr    ),
        .a_load_data_i        ( a_load_data    ),
        .d_load_we_i          ( d_load_we      ),
        .d_load_addr_i        ( d_load_addr    ),
        .d_load_data_i        ( d_load_data    ),
        .conv_w_load_we_i     ( 1'b0           ),
        .conv_w_load_addr_i   ( 6'd0           ),
        .conv_w_load_data_i   ( 32'd0          ),
        .pmu_energy_proxy_i   ( 32'd0          ),
        .pmu_cpu_sleep_cyc_i  ( 32'd0          ),
        .pmu_overlap_cyc_i    ( overlap_cyc_q  ),
        .pmu_cnt_clr_o        ( pmu_cnt_clr    ),
        .pmu_eng_a_active_o   ( pmu_eng_a_act  ),
        .pmu_eng_b_active_o   ( pmu_eng_b_act  ),
        .pmu_sram_rw_active_o ( pmu_sram_act   ),
        .busy_o               ( busy_o         ),
        .done_o               ( done_o         ),
        .pred_class_o         ( pred_class_o   )
    );

    // CSR Helper Tasks
    task automatic csr_write(input logic [7:0] addr, input logic [31:0] data);
        @(negedge clk);
        csr_req   = 1'b1;
        csr_we    = 1'b1;
        csr_addr  = addr;
        csr_wdata = data;
        @(posedge clk);
        @(negedge clk);
        csr_req   = 1'b0;
        csr_we    = 1'b0;
    endtask

    task automatic csr_read(input logic [7:0] addr, output logic [31:0] data);
        @(negedge clk);
        csr_req  = 1'b1;
        csr_we   = 1'b0;
        csr_addr = addr;
        #1;
        data = csr_rdata;
        @(posedge clk);
        @(negedge clk);
        csr_req  = 1'b0;
    endtask

    task automatic wait_done(output int cycles_waited);
        int timeout = 50000;
        cycles_waited = 0;
        while (!done_o && cycles_waited < timeout) begin
            @(posedge clk);
            cycles_waited++;
        end
        if (cycles_waited >= timeout) begin
            $fatal(1, "[FAIL] Timeout waiting for HWPE done_o!");
        end
    endtask

    int errors = 0;

    initial begin
        logic [31:0] r_tot1, r_act1, r_cls1;
        logic [31:0] r_tot2, r_act2, r_cls2, r_stat2;
        logic [31:0] r_tot3, r_act3, r_cskip3, r_sskip3, r_sram3, r_cls3;
        int cyc_w;

        csr_req = 0; csr_we = 0; csr_addr = 0; csr_wdata = 0;
        stream_we = 0; stream_data = 0;
        w_load_we = 0; w_load_addr = 0; w_load_data = 0;
        bias_load_we = 0; bias_load_addr = 0; bias_load_data = 0;
        a_load_we = 0; a_load_addr = 0; a_load_data = 0;
        d_load_we = 0; d_load_addr = 0; d_load_data = 0;

        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        $display("======================================================================");
        $display("  CHRONOS TB 4: FULL-SYSTEM 4+1 LOG-SSM HWPE & Q1 ABLATION SUITE");
        $display("======================================================================");

        // --------------------------------------------------------------------
        // 1. Load Deterministic Packed Log8 Weights into Bank 2
        //    Format {sign, k[2:0], m[3:0]} -> 8'h10 = +2.0, 8'h90 = -2.0, 8'h00 = +1.0
        // --------------------------------------------------------------------
        for (int addr = 0; addr < 1200; addr++) begin
            @(negedge clk);
            w_load_we   = 1'b1;
            w_load_addr = 11'(addr);
            if (addr >= 512 && addr < 768) begin
                // Delta projection weights: make half channels negative (small Δ -> skippable!)
                // and half channels positive (active Δ -> non-skipped!)
                w_load_data = (addr[2] == 1'b0) ? 32'h98_98_98_98 : 32'h18_18_18_18;
            end else if (addr >= 1024) begin
                // Classifier FC weights: strongly favor Class 2 (Ventricular Arrhythmia!)
                w_load_data = (addr[0] == 1'b0) ? 32'h00_28_00_80 : 32'h80_28_80_00;
            end else begin
                w_load_data = 32'h10_90_08_00;
            end
            @(posedge clk);
        end
        @(negedge clk); w_load_we = 1'b0;

        // 2. Load Biases (Bank 3) & Diagonal A_log (Bank 3) & D_skip/S4D Delta
        for (int b_a = 0; b_a < 128; b_a++) begin
            @(negedge clk);
            bias_load_we   = 1'b1;
            bias_load_addr = 7'(b_a);
            if (b_a >= 64 && b_a < 96) begin
                // Delta bias (pre-requant >>> 6):
                //   Even groups: -1536 >>> 6 = -24 (-1.5 Q4.4) -> Softplus(-1.5) ≈ 0.20 (13 Q2.6 < tau=24 -> SKIP!)
                //   Odd  groups: +1536 >>> 6 = +24 (+1.5 Q4.4) -> Softplus(+1.5) ≈ 1.70 (109 Q2.6 >= tau -> ACTIVE!)
                bias_load_data = (b_a[0] == 1'b0) ?
                    {32'(-32'sd1536), 32'(-32'sd1536), 32'(-32'sd1536), 32'(-32'sd1536)} :
                    {32'(32'sd1536),  32'(32'sd1536),  32'(32'sd1536),  32'(32'sd1536)};
            end else if (b_a >= 112) begin
                // Classifier bias: boost Class 2 (Ventricular Arrhythmia) strongly!
                bias_load_data = (b_a == 112) ?
                    {32'sd10, 32'sd4000, 32'sd20, 32'sd15} :
                    {32'sd5,  32'sd5,    32'sd5,  32'sd5};
            end else begin
                bias_load_data = 128'd0;
            end
            @(posedge clk);
        end
        @(negedge clk); bias_load_we = 1'b0;

        for (int s_a = 0; s_a < 128; s_a++) begin
            @(negedge clk);
            a_load_we   = 1'b1;
            a_load_addr = 9'(s_a);
            // Diagonal HiPPO-like decay rates: log2(|A|*16) in Q3.6
            a_load_data = {1'b0, 9'(256 + (s_a[1:0] * 48))};
            @(posedge clk);
        end
        @(negedge clk); a_load_we = 1'b0;

        for (int d_a = 0; d_a < 16; d_a++) begin
            @(negedge clk);
            d_load_we   = 1'b1;
            d_load_addr = 6'(d_a);
            d_load_data = {8'd16, 8'd4}; // Static S4D delta = 0.25, D_skip = 0.25
            @(posedge clk);
        end
        @(negedge clk); d_load_we = 1'b0;

        // 3. Stream an 8-Timestep ECG Sequence (L=8, D_in=8 => 64 samples) into Bank 0 Ring
        for (int t = 0; t < 8; t++) begin
            for (int f = 0; f < 8; f++) begin
                @(negedge clk);
                stream_we   = 1'b1;
                stream_data = 8'((t == 3 || t == 4) ? (24 + f * 4) : (6 + f));
                @(posedge clk);
            end
        end
        @(negedge clk); stream_we = 1'b0;

        // Configure Geometry: D=16, N=4, D_in=8, NUM_CLS=5, SEQ_LEN=8
        csr_write(CSR_SSM_CFG, {8'd5, 8'd8, 8'd4, 8'd16});
        csr_write(CSR_SEQ_LEN, {16'd0, 16'd8});
        // Enable Anomaly Wakeup IRQ for Classes 1..4 (bitmask 0x1E = arrhythmia beats!)
        csr_write(CSR_IRQ_CTRL, {16'd0, 8'h1E, 6'd0, 1'b0, 1'b1});

        // ====================================================================
        // RUN 1: Static S4D Baseline (mode_s6=0, skip=0)
        // ====================================================================
        csr_write(CSR_CTRL, 32'b0100_1000); // state_rst=1, lyap_en=1, mode_s6=0
        csr_write(CSR_CTRL, 32'b0100_0001); // START=1, mode_s6=0
        wait_done(cyc_w);
        csr_read(CSR_TOTAL_CYCLES,  r_tot1);
        csr_read(CSR_ACTIVE_CYCLES, r_act1);
        csr_read(CSR_CLASS_OUT,     r_cls1);
        $display("[RUN 1 - S4D Static (mode_s6=0)]  TotalCyc=%0d, EngB_Active=%0d, PredCls=%0d",
                 r_tot1, r_act1, r_cls1[3:0]);

        // ====================================================================
        // RUN 2: Selective Tiny-Mamba Full (mode_s6=1, skip=0) + Anomaly IRQ
        // ====================================================================
        csr_write(CSR_IRQ_CTRL, {16'd0, 8'h1E, 6'd0, 1'b1, 1'b1}); // Clear prior IRQ
        csr_write(CSR_CTRL, 32'b0110_1000); // state_rst=1, mode_s6=1, lyap_en=1
        csr_write(CSR_CTRL, 32'b0110_0001); // START=1, mode_s6=1
        wait_done(cyc_w);
        csr_read(CSR_TOTAL_CYCLES,  r_tot2);
        csr_read(CSR_ACTIVE_CYCLES, r_act2);
        csr_read(CSR_CLASS_OUT,     r_cls2);
        csr_read(CSR_STATUS,        r_stat2);
        $display("[RUN 2 - Mamba Full (mode_s6=1)]  TotalCyc=%0d, EngB_Active=%0d, PredCls=%0d, IRQ=%0b",
                 r_tot2, r_act2, r_cls2[3:0], irq_o);

        if (r_cls2[3:0] !== 4'd2) begin
            $display("[FAIL] Expected PredCls=2 (Arrhythmia), got %0d!", r_cls2[3:0]);
            errors++;
        end
        if (!irq_o) begin
            $display("[FAIL] Anomaly Wakeup IRQ did not assert on Class 2!");
            errors++;
        end else begin
            $display("  -> Event-Driven Anomaly Wakeup IRQ Verified: ASSERTED on Class %0d!", r_cls2[3:0]);
        end

        // ====================================================================
        // RUN 3: Selective Mamba + Level-1 Coarse Step Early-Exit + Level-2 Channel Δ-Skipping
        // ====================================================================
        csr_write(CSR_SKIP_THRESH, {16'd0, 8'd28, 8'd24}); // tau_step=28, tau_chan=24
        csr_write(CSR_CTRL, 32'b1111_1000); // state_rst=1, chan_skip=1, lyap=1, mode_s6=1, step_skip=1
        csr_write(CSR_CTRL, 32'b1111_0001); // START=1 with Dual-Level Skipping ENABLED!
        wait_done(cyc_w);
        csr_read(CSR_TOTAL_CYCLES,   r_tot3);
        csr_read(CSR_ACTIVE_CYCLES,  r_act3);
        csr_read(CSR_CHAN_SKIP_CNT,  r_cskip3);
        csr_read(CSR_STEP_SKIP_CNT,  r_sskip3);
        csr_read(CSR_SRAM_SAVED_CNT, r_sram3);
        csr_read(CSR_CLASS_OUT,      r_cls3);

        $display("[RUN 3 - Mamba + Dual-Level Skip] TotalCyc=%0d, EngB_Active=%0d, OverlapCyc=%0d, PredCls=%0d",
                 r_tot3, r_act3, overlap_cyc_q, r_cls3[3:0]);
        $display("  -> Level-1 Silent Step EarlyEx: %0d / 8 steps", r_sskip3);
        $display("  -> Level-2 Channel Δ-Skips    : %0d channels skipped", r_cskip3);
        $display("  -> Bank-1 SRAM R/W Eliminated : %0d accesses saved!", r_sram3);
        $display("  -> Engine B Active Duty Saved : %.1f%% reduction (%0d -> %0d cycles)",
                 100.0 * ($itor(r_act2) - $itor(r_act3)) / $itor(r_act2), r_act2, r_act3);
        $display("  -> Total Latency Reduction    : %.1f%% (%0d -> %0d cycles)",
                 100.0 * ($itor(r_tot2) - $itor(r_tot3)) / $itor(r_tot2), r_tot2, r_tot3);

        if ((r_cskip3 == 0 && r_sskip3 == 0) || r_sram3 == 0) begin
            $display("[FAIL] Dual-Granularity Δ-Skipping did not save channels/SRAM accesses!");
            errors++;
        end
        if (r_act3 >= r_act2) begin
            $display("[FAIL] Engine B active cycles did not decrease with Δ-Skipping!");
            errors++;
        end
        if (r_tot3 >= r_tot2) begin
            $display("[FAIL] Total latency did not decrease with Δ-Skipping!");
            errors++;
        end

        $display("======================================================================");
        if (errors == 0)
            $display("  [PASS] FULL-SYSTEM CHRONOS HWPE & ALL 3 Q1 ABLATION RUNS PASSED!");
        else
            $display("  [FAIL] %0d ERRORS!", errors);
        $display("======================================================================");
        if (errors != 0) $fatal(1, "tb_chronos_system failed");
        $finish;
    end

endmodule : tb_chronos_system
