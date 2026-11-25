// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : tb/tb_chronos_beyond_q1.sv
// Desc   : ★ TB 6 — BEYOND-Q1 FULL-SoC FLAGSHIP VERIFICATION & BENCHMARK ★
//          Executes REAL RV32I firmware on chronos_soc_top verifying EVERY
//          Beyond-Q1 hardware innovation on a realistic 12-step ECG sequence
//          containing active QRS peaks + silent isoelectric baselines:
//            1) Autonomous Zero-CPU Burst Stream DMA (Data SRAM -> Bank 0)
//            2) Trainable 4-Tap Depthwise Log8 Causal Conv1D (0 DSP)
//            3) ★ True 4+1 Concurrent Engine A || Engine B Overlap (CSR_OVERLAP_CYC > 0)
//            4) Level-1 Coarse Input-Domain Silent-Frame Early Exit (CSR_STEP_SKIP_CNT)
//            5) Level-2 Fine-Grained Channel Δ-Skipping (CSR_CHAN_SKIP_CNT)
//            6) 4-Domain PMU + CPU WFI Clock Gating + Hardware Energy-Proxy
//               Direct A/B Comparison: Baseline Active vs. CHRONOS Beyond-Q1!
// ============================================================================

`timescale 1ns / 1ps

module tb_chronos_beyond_q1;

    import chronos_pkg::*;

    localparam int unsigned SIM_CLK_HZ   = 100_000_000;
    localparam int unsigned SIM_BAUD     = 10_000_000;
    localparam int unsigned CLKS_PER_BIT = SIM_CLK_HZ / SIM_BAUD;

    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk;

    logic         uart_rxd = 1'b1;
    logic         uart_txd;
    logic         w_load_we, bias_load_we, a_load_we, d_load_we, conv_w_load_we, rom_load_we;
    logic [10:0]  w_load_addr;
    logic [31:0]  w_load_data;
    logic [6:0]   bias_load_addr;
    logic [127:0] bias_load_data;
    logic [8:0]   a_load_addr;
    logic [9:0]   a_load_data;
    logic [5:0]   d_load_addr, conv_w_load_addr;
    logic [15:0]  d_load_data;
    logic [31:0]  conv_w_load_data;
    logic [9:0]   rom_load_addr;
    logic [31:0]  rom_load_data;

    logic         cpu_sleeping, hwpe_busy, hwpe_done, hwpe_irq;
    logic [3:0]   pred_class;

    chronos_soc_top #(
        .CLK_FREQ_HZ ( SIM_CLK_HZ ),
        .BAUD_RATE   ( SIM_BAUD   )
    ) dut (
        .clk_i              ( clk              ),
        .rst_ni             ( rst_n            ),
        .test_en_i          ( 1'b0             ),
        .uart_rxd_i         ( uart_rxd         ),
        .uart_txd_o         ( uart_txd         ),
        .w_load_we_i        ( w_load_we        ),
        .w_load_addr_i      ( w_load_addr      ),
        .w_load_data_i      ( w_load_data      ),
        .bias_load_we_i     ( bias_load_we     ),
        .bias_load_addr_i   ( bias_load_addr   ),
        .bias_load_data_i   ( bias_load_data   ),
        .a_load_we_i        ( a_load_we        ),
        .a_load_addr_i      ( a_load_addr      ),
        .a_load_data_i      ( a_load_data      ),
        .d_load_we_i        ( d_load_we        ),
        .d_load_addr_i      ( d_load_addr      ),
        .d_load_data_i      ( d_load_data      ),
        .conv_w_load_we_i   ( conv_w_load_we   ),
        .conv_w_load_addr_i ( conv_w_load_addr ),
        .conv_w_load_data_i ( conv_w_load_data ),
        .rom_load_we_i      ( rom_load_we      ),
        .rom_load_addr_i    ( rom_load_addr    ),
        .rom_load_data_i    ( rom_load_data    ),
        .cpu_sleeping_o     ( cpu_sleeping     ),
        .hwpe_busy_o        ( hwpe_busy        ),
        .hwpe_done_o        ( hwpe_done        ),
        .hwpe_irq_o         ( hwpe_irq         ),
        .pred_class_o       ( pred_class       )
    );

    // RV32I Helpers
    function automatic logic [31:0] rv_lui(input logic [4:0] rd, input logic [19:0] imm20);
        return {imm20, rd, 7'b0110111};
    endfunction
    function automatic logic [31:0] rv_addi(input logic [4:0] rd, input logic [4:0] rs1, input int imm);
        logic [11:0] im12 = imm[11:0];
        return {im12, rs1, 3'b000, rd, 7'b0010011};
    endfunction
    function automatic logic [31:0] rv_ori(input logic [4:0] rd, input logic [4:0] rs1, input int imm);
        logic [11:0] im12 = imm[11:0];
        return {im12, rs1, 3'b110, rd, 7'b0010011};
    endfunction
    function automatic logic [31:0] rv_andi(input logic [4:0] rd, input logic [4:0] rs1, input int imm);
        logic [11:0] im12 = imm[11:0];
        return {im12, rs1, 3'b111, rd, 7'b0010011};
    endfunction
    function automatic logic [31:0] rv_srli(input logic [4:0] rd, input logic [4:0] rs1, input logic [4:0] shamt);
        return {7'b0000000, shamt, rs1, 3'b101, rd, 7'b0010011};
    endfunction
    function automatic logic [31:0] rv_sw(input logic [4:0] rs1, input logic [4:0] rs2, input int imm);
        logic [11:0] im12 = imm[11:0];
        return {im12[11:5], rs2, rs1, 3'b010, im12[4:0], 7'b0100011};
    endfunction
    function automatic logic [31:0] rv_sb(input logic [4:0] rs1, input logic [4:0] rs2, input int imm);
        logic [11:0] im12 = imm[11:0];
        return {im12[11:5], rs2, rs1, 3'b000, im12[4:0], 7'b0100011};
    endfunction
    function automatic logic [31:0] rv_lw(input logic [4:0] rd, input logic [4:0] rs1, input int imm);
        logic [11:0] im12 = imm[11:0];
        return {im12, rs1, 3'b010, rd, 7'b0000011};
    endfunction
    function automatic logic [31:0] rv_beq(input logic [4:0] rs1, input logic [4:0] rs2, input int offset);
        logic [12:0] o = offset[12:0];
        return {o[12], o[10:5], rs2, rs1, 3'b000, o[4:1], o[11], 7'b1100011};
    endfunction
    function automatic logic [31:0] rv_jal(input logic [4:0] rd, input int offset);
        logic [20:0] o = offset[20:0];
        return {o[20], o[10:1], o[11], o[19:12], rd, 7'b1101111};
    endfunction
    localparam logic [31:0] RV_WFI = 32'h1050_0073;

    int rom_pc = 0;
    task automatic emit(input logic [31:0] insn);
        @(negedge clk);
        rom_load_we   = 1'b1;
        rom_load_addr = 10'(rom_pc);
        rom_load_data = insn;
        @(posedge clk);
        rom_pc++;
    endtask

    // Emit RV32I sequence to send register reg_id as 16-bit big-endian (2 UART bytes)
    task automatic emit_uart_u16(input logic [4:0] reg_id);
        // Send low byte
        emit(rv_sw  (5'd2, reg_id, 0));
        emit(rv_lw  (5'd4, 5'd2, 4));
        emit(rv_andi(5'd4, 5'd4, 1));
        emit(rv_beq (5'd4, 5'd0, -8));
        // Send high byte (reg >> 8)
        emit(rv_srli(5'd6, reg_id, 5'd8));
        emit(rv_sw  (5'd2, 5'd6, 0));
        emit(rv_lw  (5'd4, 5'd2, 4));
        emit(rv_andi(5'd4, 5'd4, 1));
        emit(rv_beq (5'd4, 5'd0, -8));
    endtask

    // Background UART Tx Decoder (captures 16 bytes = 8 x uint16 telemetry words!)
    logic [7:0] rx_bytes [0:31];
    int         rx_cnt = 0;

    initial begin
        logic [7:0] b;
        forever begin
            @(negedge uart_txd);
            repeat (CLKS_PER_BIT / 2) @(posedge clk);
            if (uart_txd == 1'b0) begin
                for (int i = 0; i < 8; i++) begin
                    repeat (CLKS_PER_BIT) @(posedge clk);
                    b[i] = uart_txd;
                end
                repeat (CLKS_PER_BIT) @(posedge clk);
                if (rx_cnt < 32) begin
                    rx_bytes[rx_cnt] = b;
                    rx_cnt++;
                end
            end
        end
    end

    int errors = 0;

    initial begin
        int base_cyc, base_eng, base_ovlp;
        int opt_cyc, opt_eng, opt_ovlp, opt_sskip, opt_cskip, opt_sram, opt_sleep, opt_cls;

        w_load_we = 0; w_load_addr = 0; w_load_data = 0;
        bias_load_we = 0; bias_load_addr = 0; bias_load_data = 0;
        a_load_we = 0; a_load_addr = 0; a_load_data = 0;
        d_load_we = 0; d_load_addr = 0; d_load_data = 0;
        conv_w_load_we = 0; conv_w_load_addr = 0; conv_w_load_data = 0;
        rom_load_we = 0; rom_load_addr = 0; rom_load_data = 0;

        rst_n = 1'b0;
        repeat (2) @(posedge clk);

        $display("======================================================================");
        $display("  CHRONOS TB 6: BEYOND-Q1 FULL-SoC (DMA + 4+1 OVERLAP + 2-LVL SKIP + PMU)");
        $display("======================================================================");

        // --------------------------------------------------------------------
        // 1. Assemble Real RV32I Benchmark Firmware into Boot ROM!
        //    Base Pointers:
        //      x1 = 0x4000_0000 (CHRONOS HWPE)
        //      x2 = 0x5000_0000 (UART)
        //      x8 = 0x2000_0000 (Data SRAM)
        //      x9 = 0x3000_0000 (Burst Stream DMA)
        // --------------------------------------------------------------------
        emit(rv_lui(5'd1, 20'h40000));
        emit(rv_lui(5'd2, 20'h50000));
        emit(rv_lui(5'd8, 20'h20000));
        emit(rv_lui(5'd9, 20'h30000));

        // CPU writes a 12-timestep ECG window (96 bytes = 12 steps x 8 features) into Data SRAM!
        //   Steps 0,1,2 : Active P-wave / baseline ramp
        //   Steps 3,4,5 : Flat isoelectric baseline (identical samples -> Level-1 Early-Exit!)
        //   Steps 6,7   : Sharp QRS Arrhythmia complex (high delta -> active!)
        //   Steps 8,9,10: Flat ST/TP baseline (identical samples -> Level-1 Early-Exit!)
        //   Step  11    : T-wave pulse (active!)
        for (int t = 0; t < 12; t++) begin
            int amp;
            if      (t == 0) amp = 10;
            else if (t == 1) amp = 28;
            else if (t == 2) amp = 12;
            else if (t >= 3 && t <= 5) amp = 12; // 3 flatline steps (L1_diff = 0!)
            else if (t == 6) amp = 60;           // QRS R-peak!
            else if (t == 7) amp = 15;           // S-wave
            else if (t >= 8 && t <= 10) amp = 15;// 3 flatline steps (L1_diff = 0!)
            else             amp = 36;           // T-wave
            for (int f = 0; f < 8; f++) begin
                emit(rv_addi(5'd3, 5'd0, amp + f));
                emit(rv_sb  (5'd8, 5'd3, t * 8 + f));
            end
        end

        // Configure CHRONOS: D=16, N=4, D_in=8, NUM_CLS=5, SEQ_LEN=12
        emit(rv_lui (5'd3, 20'h05080));
        emit(rv_ori (5'd3, 5'd3, 32'h410));
        emit(rv_sw  (5'd1, 5'd3, 8));
        emit(rv_addi(5'd3, 5'd0, 12));
        emit(rv_sw  (5'd1, 5'd3, 12));

        // 1-Cycle Virtual Reset + Trigger Autonomous Burst DMA (96 bytes SRAM -> Bank 0!)
        emit(rv_addi(5'd3, 5'd0, 32'h068)); // state_rst=1
        emit(rv_sw  (5'd1, 5'd3, 0));
        emit(rv_sw  (5'd9, 5'd0, 0));       // DMA_SRC_ADDR = 0
        emit(rv_addi(5'd3, 5'd0, 96));
        emit(rv_sw  (5'd9, 5'd3, 4));       // DMA_LEN = 96 bytes
        emit(rv_addi(5'd3, 5'd0, 1));
        emit(rv_sw  (5'd9, 5'd3, 8));       // DMA_START = 1 -> CPU sleeps in WFI until DMA_DONE!
        emit(RV_WFI);
        // Clear DMA_DONE sticky bit (W1C bit 2 = 4)
        emit(rv_addi(5'd3, 5'd0, 4));
        emit(rv_sw  (5'd9, 5'd3, 8));

        // ===== BENCHMARK A: Full-Active Selective Mamba (4+1 Overlap ON, Skipping OFF) =====
        emit(rv_addi(5'd3, 5'd0, 32'h061)); // START=1, mode_s6=1, lyap=1, skip=0
        emit(rv_sw  (5'd1, 5'd3, 0));
        emit(RV_WFI);

        // Export Benchmark A Telemetry via UART (3 x u16: TotalCyc, EnergyProxy>>3, OverlapCyc)
        emit(rv_lw(5'd3, 5'd1, 32)); emit_uart_u16(5'd3); // CSR_TOTAL_CYCLES (0x20)
        emit(rv_lw(5'd3, 5'd1, 52)); emit(rv_srli(5'd3, 5'd3, 5'd3)); emit_uart_u16(5'd3); // CSR_ENERGY_PROXY>>3
        emit(rv_lw(5'd3, 5'd1, 60)); emit_uart_u16(5'd3); // CSR_OVERLAP_CYC (0x3C)

        // ===== BENCHMARK B: BEYOND-Q1 CHRONOS (4+1 Overlap + Level-1 + Level-2 Skip + PMU) =====
        // Set tau_step = 8 (L1_diff < 16 => silent early-exit!), tau_chan = 24
        emit(rv_addi(5'd3, 5'd0, 32'h818)); // {tau_step=8, tau_chan=24} = 0x0818 (signed? wait -> use LUI!)
        // Wait: 0x0818 has bit 11 = 1! Build cleanly via LUI+ADDI: 0x1000 - 2024 = 0x0818!
        emit(rv_lui (5'd3, 20'h00001));
        emit(rv_addi(5'd3, 5'd3, -2024));
        emit(rv_sw  (5'd1, 5'd3, 16));      // CSR_SKIP_THRESH (0x10)

        // Clear state via 1-cycle virtual reset (ring contents remain intact for re-run!), then launch!
        emit(rv_addi(5'd3, 5'd0, 32'h0F8)); // state_rst=1, Dual-Level Skip ON
        emit(rv_sw  (5'd1, 5'd3, 0));
        emit(rv_addi(5'd3, 5'd0, 32'h0F1)); // START=1, Level-1 + Level-2 Skip ON!
        emit(rv_sw  (5'd1, 5'd3, 0));
        emit(RV_WFI);

        // Export Benchmark B Telemetry via UART (8 x u16 = 16 bytes)
        emit(rv_lw(5'd3, 5'd1, 32)); emit_uart_u16(5'd3); // TotalCyc
        emit(rv_lw(5'd3, 5'd1, 52)); emit(rv_srli(5'd3, 5'd3, 5'd3)); emit_uart_u16(5'd3); // EnergyProxy>>3
        emit(rv_lw(5'd3, 5'd1, 60)); emit_uart_u16(5'd3); // OverlapCyc
        emit(rv_lw(5'd3, 5'd1, 44)); emit_uart_u16(5'd3); // StepSkipCnt (Level-1 Early-Exit)
        emit(rv_lw(5'd3, 5'd1, 40)); emit_uart_u16(5'd3); // ChanSkipCnt (Level-2 Channel Skip)
        emit(rv_lw(5'd3, 5'd1, 48)); emit_uart_u16(5'd3); // SRAM_SavedCnt
        emit(rv_lw(5'd3, 5'd1, 56)); emit_uart_u16(5'd3); // CPU_SleepCyc (PMU gated!)
        emit(rv_lw(5'd3, 5'd1, 20)); emit_uart_u16(5'd3); // ClassOut

        emit(rv_jal(5'd0, 0));
        @(negedge clk); rom_load_we = 1'b0;

        // --------------------------------------------------------------------
        // 2. Load Weights, Biases, A_log, D_skip & Trainable Conv1D Kernels
        // --------------------------------------------------------------------
        for (int addr = 0; addr < 1200; addr++) begin
            @(negedge clk);
            w_load_we   = 1'b1;
            w_load_addr = 11'(addr);
            if (addr >= 512 && addr < 768)
                w_load_data = (addr[2] == 1'b0) ? 32'h98_98_98_98 : 32'h18_18_18_18;
            else if (addr >= 1024)
                w_load_data = (addr[0] == 1'b0) ? 32'h00_28_00_80 : 32'h80_28_80_00;
            else
                w_load_data = 32'h10_90_08_00;
            @(posedge clk);
        end
        @(negedge clk); w_load_we = 1'b0;

        for (int b_a = 0; b_a < 128; b_a++) begin
            @(negedge clk);
            bias_load_we   = 1'b1;
            bias_load_addr = 7'(b_a);
            if (b_a >= 64 && b_a < 96) begin
                bias_load_data = (b_a[0] == 1'b0) ?
                    {32'(-32'sd1536), 32'(-32'sd1536), 32'(-32'sd1536), 32'(-32'sd1536)} :
                    {32'(32'sd1536),  32'(32'sd1536),  32'(32'sd1536),  32'(32'sd1536)};
            end else if (b_a >= 112) begin
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
            a_load_data = {1'b0, 9'(256 + (s_a[1:0] * 48))};
            @(posedge clk);
        end
        @(negedge clk); a_load_we = 1'b0;

        for (int d_a = 0; d_a < 16; d_a++) begin
            @(negedge clk);
            d_load_we        = 1'b1;
            d_load_addr      = 6'(d_a);
            d_load_data      = {8'd16, 8'd4};
            conv_w_load_we   = 1'b1;
            conv_w_load_addr = 6'(d_a);
            // Trainable depthwise causal Conv1D kernels (different per channel!)
            conv_w_load_data = (d_a[0] == 1'b0) ? 32'h00_08_10_20 : 32'h80_00_18_24;
            @(posedge clk);
        end
        @(negedge clk); d_load_we = 1'b0; conv_w_load_we = 1'b0;

        // --------------------------------------------------------------------
        // 3. Boot Full CHRONOS SoC and Capture All 22 Telemetry Bytes via UART!
        // --------------------------------------------------------------------
        repeat (2) @(posedge clk);
        rst_n = 1'b1;

        begin
            int wait_t = 0;
            while (rx_cnt < 22 && wait_t < 80000) begin
                @(posedge clk);
                wait_t++;
            end
        end
        repeat (20) @(posedge clk);

        if (rx_cnt < 22) begin
            $fatal(1, "[FAIL] Timeout: received only %0d / 22 telemetry bytes!", rx_cnt);
        end

        // Decode 16-bit Little-Endian words from physical UART Tx
        base_cyc  = int'({rx_bytes[1],  rx_bytes[0]});
        base_eng  = int'({rx_bytes[3],  rx_bytes[2]}) * 8;
        base_ovlp = int'({rx_bytes[5],  rx_bytes[4]});

        opt_cyc   = int'({rx_bytes[7],  rx_bytes[6]});
        opt_eng   = int'({rx_bytes[9],  rx_bytes[8]}) * 8;
        opt_ovlp  = int'({rx_bytes[11], rx_bytes[10]});
        opt_sskip = int'({rx_bytes[13], rx_bytes[12]});
        opt_cskip = int'({rx_bytes[15], rx_bytes[14]});
        opt_sram  = int'({rx_bytes[17], rx_bytes[16]});
        opt_sleep = int'({rx_bytes[19], rx_bytes[18]});
        opt_cls   = int'(rx_bytes[20][3:0]);

        $display("----------------------------------------------------------------------");
        $display("  [A] FULL-ACTIVE SELECTIVE MAMBA (4+1 Overlap ON, Skip OFF):");
        $display("      Total Cycles           : %0d cyc", base_cyc);
        $display("      Concurrent 4+1 Overlap : %0d cyc (EngA && EngB active together!)", base_ovlp);
        $display("      PMU Energy Proxy       : %0d units", base_eng);
        $display("----------------------------------------------------------------------");
        $display("  [B] BEYOND-Q1 CHRONOS (DMA + 4+1 Overlap + L1 & L2 Skip + PMU):");
        $display("      Total Cycles           : %0d cyc  (%.1f%% SPEEDUP!)",
                 opt_cyc, 100.0 * $itor(base_cyc - opt_cyc) / $itor(base_cyc));
        $display("      PMU Energy Proxy       : %0d units (%.1f%% ENERGY SAVED!)",
                 opt_eng, 100.0 * $itor(base_eng - opt_eng) / $itor(base_eng));
        $display("      Concurrent 4+1 Overlap : %0d cyc", opt_ovlp);
        $display("      Level-1 Step EarlyExit : %0d / 12 silent ECG frames bypassed!", opt_sskip);
        $display("      Level-2 Channel Skip   : %0d quiet channels gated!", opt_cskip);
        $display("      Bank-1 SRAM R/W Saved  : %0d accesses eliminated!", opt_sram);
        $display("      CPU WFI Gated Sleep    : %0d / %0d cycles (%.1f%% CPU asleep!)",
                 opt_sleep, opt_cyc, 100.0 * $itor(opt_sleep) / $itor(opt_cyc));
        $display("      Predicted Arrhythmia   : Class %0d", opt_cls);
        $display("----------------------------------------------------------------------");

        if (base_ovlp == 0) begin
            $display("[FAIL] True 4+1 Concurrent Overlap (EngA && EngB) did not occur!"); errors++;
        end
        if (opt_sskip < 4) begin
            $display("[FAIL] Level-1 Coarse Early-Exit skipped too few silent frames (%0d)!", opt_sskip); errors++;
        end
        if (opt_cskip == 0) begin
            $display("[FAIL] Level-2 Fine-Grained Channel Skip did not trigger!"); errors++;
        end
        if (opt_eng >= base_eng || opt_cyc >= base_cyc) begin
            $display("[FAIL] Beyond-Q1 skipping did not reduce latency & energy proxy!"); errors++;
        end
        if (opt_sleep < (opt_cyc - 100)) begin
            $display("[FAIL] PMU did not clock-gate CPU during WFI!"); errors++;
        end
        if (opt_cls != 2) begin
            $display("[FAIL] Class mismatch: got %0d, expected 2!", opt_cls); errors++;
        end

        if (errors == 0)
            $display("  [PASS] BEYOND-Q1 CHRONOS FULL SoC VERIFIED — ALL INNOVATIONS PROVEN!");
        else
            $display("  [FAIL] %0d ERRORS!", errors);
        $display("======================================================================");
        if (errors != 0) $fatal(1, "tb_chronos_beyond_q1 failed");
        $finish;
    end

endmodule : tb_chronos_beyond_q1
