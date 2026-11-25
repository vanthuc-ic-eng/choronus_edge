// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Edge AI SoC
// File   : tb/tb_chronos_soc.sv
// Desc   : ★ TB 5 — FULL END-TO-END RISC-V SoC FIRMWARE EXECUTION TEST ★
//          Executes REAL RV32I machine code loaded into Boot ROM (0x0000_0000):
//            1) RISC-V CPU configures CHRONOS HWPE CSRs via SW instructions
//            2) CPU enables UART Rx Auto-Stream DMA & receives live ECG stream
//               directly over physical uart_rxd_i into Bank 0 (ZERO CPU copy!)
//            3) CPU launches Selective Mamba + Dual-Granularity Δ-Skipping,
//               then executes WFI (32'h1050_0073) to enter LOW-POWER SLEEP!
//            4) CHRONOS detects Class 2 Arrhythmia, asserts IRQ -> WAKES CPU!
//            5) CPU wakes up, reads PredCls + ChanSkipCnt + SRAM_SavedCnt via LW
//               and transmits a 3-byte binary telemetry frame on uart_txd_o!
//            6) Testbench decodes physical uart_txd_o waveform bit-by-bit and
//               verifies 100% end-to-end SoC correctness + WFI sleep savings!
// ============================================================================

`timescale 1ns / 1ps

module tb_chronos_soc;

    import chronos_pkg::*;

    // Use fast simulation baud (10 Mbaud @ 100 MHz => 10 clocks/bit) for crisp TB speed
    localparam int unsigned SIM_CLK_HZ   = 100_000_000;
    localparam int unsigned SIM_BAUD     = 10_000_000;
    localparam int unsigned CLKS_PER_BIT = SIM_CLK_HZ / SIM_BAUD;

    logic clk   = 1'b0;
    logic rst_n = 1'b0;
    always #5 clk = ~clk; // 100 MHz

    logic        uart_rxd = 1'b1;
    logic        uart_txd;
    logic        w_load_we, bias_load_we, a_load_we, d_load_we, rom_load_we;
    logic [10:0] w_load_addr;
    logic [31:0] w_load_data;
    logic [6:0]  bias_load_addr;
    logic [127:0] bias_load_data;
    logic [8:0]  a_load_addr;
    logic [9:0]  a_load_data;
    logic [5:0]  d_load_addr;
    logic [15:0] d_load_data;
    logic [9:0]  rom_load_addr;
    logic [31:0] rom_load_data;

    logic        cpu_sleeping, hwpe_busy, hwpe_done, hwpe_irq;
    logic [3:0]  pred_class;

    chronos_soc_top #(
        .CLK_FREQ_HZ ( SIM_CLK_HZ ),
        .BAUD_RATE   ( SIM_BAUD   )
    ) dut (
        .clk_i            ( clk            ),
        .rst_ni           ( rst_n          ),
        .test_en_i        ( 1'b0           ),
        .uart_rxd_i       ( uart_rxd       ),
        .uart_txd_o       ( uart_txd       ),
        .w_load_we_i      ( w_load_we      ),
        .w_load_addr_i    ( w_load_addr    ),
        .w_load_data_i    ( w_load_data    ),
        .bias_load_we_i   ( bias_load_we   ),
        .bias_load_addr_i ( bias_load_addr ),
        .bias_load_data_i ( bias_load_data ),
        .a_load_we_i      ( a_load_we      ),
        .a_load_addr_i    ( a_load_addr    ),
        .a_load_data_i    ( a_load_data    ),
        .d_load_we_i      ( d_load_we      ),
        .d_load_addr_i    ( d_load_addr    ),
        .d_load_data_i    ( d_load_data    ),
        .conv_w_load_we_i   ( 1'b0         ),
        .conv_w_load_addr_i ( 6'd0         ),
        .conv_w_load_data_i ( 32'd0        ),
        .rom_load_we_i    ( rom_load_we    ),
        .rom_load_addr_i  ( rom_load_addr  ),
        .rom_load_data_i  ( rom_load_data  ),
        .cpu_sleeping_o   ( cpu_sleeping   ),
        .hwpe_busy_o      ( hwpe_busy      ),
        .hwpe_done_o      ( hwpe_done      ),
        .hwpe_irq_o       ( hwpe_irq       ),
        .pred_class_o     ( pred_class     )
    );

    // ------------------------------------------------------------------------
    // Minimal RV32I Instruction Encoder Helpers (Zero External Toolchain Needed!)
    // ------------------------------------------------------------------------
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

    // Helper: Write 1 word into Boot ROM
    int rom_pc = 0;
    task automatic emit(input logic [31:0] insn);
        @(negedge clk);
        rom_load_we   = 1'b1;
        rom_load_addr = 10'(rom_pc);
        rom_load_data = insn;
        @(posedge clk);
        rom_pc++;
    endtask

    // Helper: Send 1 byte over physical uart_rxd pin (8-N-1)
    task automatic uart_send_byte(input logic [7:0] b);
        uart_rxd = 1'b0; // Start bit
        repeat (CLKS_PER_BIT) @(posedge clk);
        for (int i = 0; i < 8; i++) begin
            uart_rxd = b[i];
            repeat (CLKS_PER_BIT) @(posedge clk);
        end
        uart_rxd = 1'b1; // Stop bit
        repeat (CLKS_PER_BIT) @(posedge clk);
    endtask

    // Background UART Tx Decoder Monitor
    logic [7:0] rx_bytes [0:7];
    int         rx_cnt = 0;
    int         sleep_cycles = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sleep_cycles <= 0;
        else if (cpu_sleeping) sleep_cycles <= sleep_cycles + 1;
    end

    // Monitor physical uart_txd pin asynchronously
    initial begin
        logic [7:0] b;
        forever begin
            @(negedge uart_txd);
            // Wait half bit to sample start bit center
            repeat (CLKS_PER_BIT / 2) @(posedge clk);
            if (uart_txd == 1'b0) begin
                for (int i = 0; i < 8; i++) begin
                    repeat (CLKS_PER_BIT) @(posedge clk);
                    b[i] = uart_txd;
                end
                repeat (CLKS_PER_BIT) @(posedge clk); // Stop bit
                if (rx_cnt < 8) begin
                    rx_bytes[rx_cnt] = b;
                    rx_cnt++;
                end
            end
        end
    end

    int errors = 0;

    initial begin
        w_load_we = 0; w_load_addr = 0; w_load_data = 0;
        bias_load_we = 0; bias_load_addr = 0; bias_load_data = 0;
        a_load_we = 0; a_load_addr = 0; a_load_data = 0;
        d_load_we = 0; d_load_addr = 0; d_load_data = 0;
        rom_load_we = 0; rom_load_addr = 0; rom_load_data = 0;

        // Hold CPU in reset while loading Boot ROM & Model Parameters
        rst_n = 1'b0;
        repeat (2) @(posedge clk);

        $display("======================================================================");
        $display("  CHRONOS TB 5: FULL-SoC RISC-V RV32I + WFI + UART + MAMBA-HWPE TEST");
        $display("======================================================================");

        // --------------------------------------------------------------------
        // 1. Assemble Real RV32I Firmware into Boot ROM (0x0000_0000)
        //    Register convention:
        //      x1 = 0x4000_0000 (CHRONOS HWPE base)
        //      x2 = 0x5000_0000 (UART base)
        //      x3 = scratch / data
        //      x4 = status poll
        // --------------------------------------------------------------------
        emit(rv_lui (5'd1, 20'h40000));             // x1 = 0x4000_0000 (HWPE)
        emit(rv_lui (5'd2, 20'h50000));             // x2 = 0x5000_0000 (UART)

        // 1-Cycle Virtual State & Ring Reset BEFORE UART ECG stream starts!
        emit(rv_addi(5'd3, 5'd0, 32'h0F8));         // state_rst=1, chan_skip=1, lyap=1, mode_s6=1, step_skip=1
        emit(rv_sw  (5'd1, 5'd3, 0));

        // Enable UART Hardware Auto-Stream into Bank 0 (0x5000_0008 = 1)
        emit(rv_addi(5'd3, 5'd0, 1));
        emit(rv_sw  (5'd2, 5'd3, 8));

        // Set CSR_SSM_CFG = {5, 8, 4, 16} = 0x0508_0410
        emit(rv_lui (5'd3, 20'h05080));
        emit(rv_ori (5'd3, 5'd3, 32'h410));
        emit(rv_sw  (5'd1, 5'd3, 8));               // CSR_SSM_CFG (0x08)

        // CSR_SEQ_LEN = 8 timesteps (0x0C)
        emit(rv_addi(5'd3, 5'd0, 8));
        emit(rv_sw  (5'd1, 5'd3, 12));

        // CSR_SKIP_THRESH: tau_step=28, tau_chan=24 -> 0x1C18 (0x10)
        emit(rv_lui (5'd3, 20'h00002));
        emit(rv_addi(5'd3, 5'd3, -1000));           // 0x2000 - 1000 = 0x1C18!
        emit(rv_sw  (5'd1, 5'd3, 16));

        // CSR_IRQ_CTRL: irq_en=1, anomaly_mask=0x1E -> 0x1E01 (0x18)
        emit(rv_lui (5'd3, 20'h00002));
        emit(rv_addi(5'd3, 5'd3, -511));            // 0x2000 - 511 = 0x1E01!
        emit(rv_sw  (5'd1, 5'd3, 24));

        // Wait loop: x5 = 900 iterations (~10,800 cycles so all 64 UART bytes finish arriving)
        emit(rv_addi(5'd5, 5'd0, 900));
        // loop_wait:
        emit(rv_addi(5'd5, 5'd5, -1));
        emit(rv_beq (5'd5, 5'd0, 8));               // if x5==0 skip next JAL
        emit(rv_jal (5'd0, -8));                    // jump back to addi x5,-1

        // Launch CHRONOS Mamba w/ Level-2 Channel Δ-Skipping (keep step_skip=0 here for exact 64ch count!)
        emit(rv_addi(5'd3, 5'd0, 32'h0E1));         // START=1, chan_skip=1, lyap=1, mode_s6=1, step_skip=0!
        emit(rv_sw  (5'd1, 5'd3, 0));

        // ★ EXECUTE WFI — CPU ENTERS LOW-POWER SLEEP UNTIL CHRONOS IRQ WAKES IT! ★
        emit(RV_WFI);

        // --- CPU WOKEN UP BY HARDWARE EVENT IRQ! ---
        // Read Byte 0: 0xA0 | pred_class (from CSR_CLASS_OUT @ 0x14)
        emit(rv_lw  (5'd3, 5'd1, 20));
        emit(rv_andi(5'd3, 5'd3, 15));
        emit(rv_ori (5'd3, 5'd3, 32'h0A0));
        emit(rv_sw  (5'd2, 5'd3, 0));               // UART TX Byte 0

        // Poll UART TX Ready (bit 0 of 0x5000_0004)
        emit(rv_lw  (5'd4, 5'd2, 4));
        emit(rv_andi(5'd4, 5'd4, 1));
        emit(rv_beq (5'd4, 5'd0, -8));

        // Read Byte 1: CSR_CHAN_SKIP_CNT[7:0] (from 0x28)
        emit(rv_lw  (5'd3, 5'd1, 40));
        emit(rv_sw  (5'd2, 5'd3, 0));               // UART TX Byte 1

        // Poll UART TX Ready
        emit(rv_lw  (5'd4, 5'd2, 4));
        emit(rv_andi(5'd4, 5'd4, 1));
        emit(rv_beq (5'd4, 5'd0, -8));

        // Read Byte 2: CSR_SRAM_SAVED_CNT >> 2 (from 0x30: 512 >> 2 = 128 = 0x80)
        emit(rv_lw  (5'd3, 5'd1, 48));
        emit(rv_srli(5'd3, 5'd3, 5'd2));
        emit(rv_sw  (5'd2, 5'd3, 0));               // UART TX Byte 2

        // Halt cleanly in infinite WFI/loop after reporting
        emit(rv_jal (5'd0, 0));

        @(negedge clk); rom_load_we = 1'b0;

        // --------------------------------------------------------------------
        // 2. Load Model Weights, Biases, A_log, D_skip into TCDM Banks 2 & 3
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
            d_load_we   = 1'b1;
            d_load_addr = 6'(d_a);
            d_load_data = {8'd16, 8'd4};
            @(posedge clk);
        end
        @(negedge clk); d_load_we = 1'b0;

        // --------------------------------------------------------------------
        // 3. Release SoC Reset! CPU Boots, Enables UART Auto-Stream, We Stream ECG!
        // --------------------------------------------------------------------
        repeat (2) @(posedge clk);
        rst_n = 1'b1;

        // Wait 80 cycles for CPU to execute boot instructions & enable UART Auto-Stream
        repeat (80) @(posedge clk);

        // Stream 64 ECG samples (8 timesteps x 8 features) over physical uart_rxd pin!
        // ZERO CPU intervention — UART Auto-Stream DMA pushes straight into Bank 0!
        for (int t = 0; t < 8; t++) begin
            for (int f = 0; f < 8; f++) begin
                uart_send_byte(8'((t == 3 || t == 4) ? (24 + f * 4) : (6 + f)));
            end
        end

        // Wait for CPU to wake from WFI and transmit all 3 telemetry bytes over uart_txd!
        begin
            int wait_t = 0;
            while (rx_cnt < 3 && wait_t < 40000) begin
                @(posedge clk);
                wait_t++;
            end
        end

        // Extra settle cycles
        repeat (20) @(posedge clk);

        $display("[SOC VERIFY] UART Tx Telemetry Decoded (%0d bytes):", rx_cnt);
        if (rx_cnt < 3) begin
            $display("[FAIL] Expected 3 UART telemetry bytes, received %0d!", rx_cnt);
            errors++;
        end else begin
            $display("  -> Byte 0 (Header|PredCls) : 0x%02h (PredCls = %0d, Expected 0xA2)",
                     rx_bytes[0], rx_bytes[0][3:0]);
            $display("  -> Byte 1 (ChanSkipCnt)    : %0d channels skipped (Expected 64)",
                     rx_bytes[1]);
            $display("  -> Byte 2 (SRAM_Saved>>2)  : %0d -> %0d Bank-1 accesses saved (Expected 512)",
                     rx_bytes[2], int'(rx_bytes[2]) * 4);
            $display("  -> RISC-V WFI Sleep Cycles : %0d cycles slept during HWPE run!",
                     sleep_cycles);
            $display("  -> Anomaly Wakeup IRQ Pin  : %0b", hwpe_irq);

            if (rx_bytes[0] !== 8'hA2) begin
                $display("[FAIL] Byte 0 mismatch!"); errors++;
            end
            if (rx_bytes[1] !== 8'd64) begin
                $display("[FAIL] Byte 1 (ChanSkipCnt) mismatch!"); errors++;
            end
            if (rx_bytes[2] !== 8'd128) begin
                $display("[FAIL] Byte 2 (SRAM_Saved>>2) mismatch!"); errors++;
            end
            if (sleep_cycles < 1000) begin
                $display("[FAIL] RISC-V CPU did not sleep in WFI during HWPE execution!"); errors++;
            end
        end

        $display("======================================================================");
        if (errors == 0)
            $display("  [PASS] FULL CHRONOS-PULP RISC-V EDGE AI SoC PASSED END-TO-END!");
        else
            $display("  [FAIL] %0d ERRORS!", errors);
        $display("======================================================================");
        if (errors != 0) $fatal(1, "tb_chronos_soc failed");
        $finish;
    end

endmodule : tb_chronos_soc
