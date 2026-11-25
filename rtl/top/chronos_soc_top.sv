// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/top/chronos_soc_top.sv
// Desc   : ★ COMPLETE BEYOND-Q1 FLAGSHIP EDGE AI SYSTEM-ON-CHIP ★
//          Integrates:
//            1) Host RISC-V RV32I w/ WFI Sleep + PMU Glitch-Free Clock Gating
//            2) 4-Domain Power Management Unit (`chronos_pmu`) w/ Real-Time
//               Energy-Proxy, WFI Sleep, & Concurrent Overlap Telemetry
//            3) 5-Slave Registered Crossbar (`chronos_soc_bus`)
//            4) 4 KB Dual-Port Boot ROM + 4 KB True Dual-Port Data SRAM
//            5) Autonomous Zero-CPU Burst Stream DMA (`chronos_dma_ctrl` @ 0x3000_0000)
//            6) Full-Duplex UART Rx/Tx w/ Auto-Stream DMA (@ 0x5000_0000)
//            7) CHRONOS 4+1 Concurrent Zero-DSP Log-SSM HWPE (@ 0x4000_0000):
//               - Level-1 Coarse Input-Domain Silent-Frame Early Exit
//               - Trainable 4-Tap Log8 Causal Conv1D (0 DSP)
//               - True 4+1 Concurrent Engine A || Engine B Overlap
//               - Level-2 Fine-Grained Channel Δ-Skip + 0-Bubble DP State SRAM
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_soc_top #(
    parameter              BOOT_HEX    = "",
    parameter int unsigned CLK_FREQ_HZ = 100_000_000,
    parameter int unsigned BAUD_RATE   = 115_200
)(
    input  logic         clk_i,
    input  logic         rst_ni,
    input  logic         test_en_i,

    // Full-Duplex Serial Interface
    input  logic         uart_rxd_i,
    output logic         uart_txd_o,

    // Parameter / Boot Load Ports (Weights, Bias, A, D, Trainable Conv1D, Boot ROM)
    input  logic         w_load_we_i,
    input  logic [10:0]  w_load_addr_i,
    input  logic [31:0]  w_load_data_i,

    input  logic         bias_load_we_i,
    input  logic [6:0]   bias_load_addr_i,
    input  logic [127:0] bias_load_data_i,

    input  logic         a_load_we_i,
    input  logic [8:0]   a_load_addr_i,
    input  logic [9:0]   a_load_data_i,

    input  logic         d_load_we_i,
    input  logic [5:0]   d_load_addr_i,
    input  logic [15:0]  d_load_data_i,

    input  logic         conv_w_load_we_i,
    input  logic [5:0]   conv_w_load_addr_i,
    input  logic [31:0]  conv_w_load_data_i,

    input  logic         rom_load_we_i,
    input  logic [9:0]   rom_load_addr_i,
    input  logic [31:0]  rom_load_data_i,

    // SoC Status & Power Observability Pins
    output logic         cpu_sleeping_o,
    output logic         hwpe_busy_o,
    output logic         hwpe_done_o,
    output logic         hwpe_irq_o,
    output logic [3:0]   pred_class_o
);

    // 1. Async-Assert Sync-Deassert Reset Synchronizer
    logic rst_meta_n, rst_sync_n;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin rst_meta_n <= 1'b0; rst_sync_n <= 1'b0; end
        else         begin rst_meta_n <= 1'b1; rst_sync_n <= rst_meta_n; end
    end

    // 2. 4-Domain Power Management Unit (Gates CPU clock cleanly during WFI!)
    logic        clk_cpu_gated;
    logic        pmu_cnt_clr, pmu_eng_a_act, pmu_eng_b_act, pmu_sram_act;
    logic [31:0] pmu_energy_proxy, pmu_cpu_sleep_cyc, pmu_overlap_cyc;
    logic        dma_done_irq;
    logic        cpu_wakeup_event;

    assign cpu_wakeup_event = hwpe_irq_o | hwpe_done_o | dma_done_irq;

    chronos_pmu u_pmu (
        .clk_i            ( clk_i             ),
        .rst_ni           ( rst_sync_n        ),
        .test_en_i        ( test_en_i         ),
        .cnt_clr_i        ( pmu_cnt_clr       ),
        .cpu_sleeping_i   ( cpu_sleeping_o    ),
        .cpu_wakeup_i     ( cpu_wakeup_event  ),
        .eng_a_active_i   ( pmu_eng_a_act     ),
        .eng_b_active_i   ( pmu_eng_b_act     ),
        .sram_rw_active_i ( pmu_sram_act      ),
        .hwpe_busy_i      ( hwpe_busy_o       ),
        .clk_cpu_gated_o  ( clk_cpu_gated     ),
        .energy_proxy_o   ( pmu_energy_proxy  ),
        .cpu_sleep_cyc_o  ( pmu_cpu_sleep_cyc ),
        .overlap_cyc_o    ( pmu_overlap_cyc   )
    );

    // 3. Host RISC-V RV32I Core (Clocked by PMU Gated Domain 0!)
    logic [31:0] cpu_iaddr, cpu_idata;
    logic        cpu_ireq, cpu_ivalid;
    logic [31:0] cpu_daddr, cpu_dwdata, cpu_drdata;
    logic [3:0]  cpu_dbe;
    logic        cpu_dwe, cpu_dreq, cpu_dvalid;
    logic        irq_ack_unused;

    chronos_riscv_rv32i #(.BOOT_ADDR ( 32'h0000_0000 )) u_riscv_cpu (
        .clk_i          ( clk_cpu_gated    ),
        .rst_ni         ( rst_sync_n       ),
        .instr_addr_o   ( cpu_iaddr        ),
        .instr_req_o    ( cpu_ireq         ),
        .instr_data_i   ( cpu_idata        ),
        .instr_valid_i  ( cpu_ivalid       ),
        .data_addr_o    ( cpu_daddr        ),
        .data_wdata_o   ( cpu_dwdata       ),
        .data_be_o      ( cpu_dbe          ),
        .data_we_o      ( cpu_dwe          ),
        .data_req_o     ( cpu_dreq         ),
        .data_rdata_i   ( cpu_drdata       ),
        .data_valid_i   ( cpu_dvalid       ),
        .wakeup_irq_i   ( cpu_wakeup_event ),
        .cpu_sleeping_o ( cpu_sleeping_o   ),
        .irq_ack_o      ( irq_ack_unused   )
    );

    // 4. 5-Slave Crossbar Interconnect
    logic [11:0] rom_daddr;
    logic        rom_dreq, rom_dvalid;
    logic [31:0] rom_drdata;

    logic [11:0] ram_addr;
    logic [31:0] ram_wdata, ram_rdata;
    logic [3:0]  ram_be;
    logic        ram_we, ram_req, ram_valid;

    logic [3:0]  dma_addr;
    logic [31:0] dma_wdata, dma_rdata;
    logic        dma_we, dma_req, dma_valid;

    logic [7:0]  hwpe_addr;
    logic [31:0] hwpe_wdata, hwpe_rdata_comb, hwpe_rdata_q;
    logic        hwpe_we, hwpe_req, hwpe_valid_q;

    logic [3:0]  uart_addr;
    logic [31:0] uart_wdata, uart_rdata;
    logic        uart_we, uart_req, uart_valid;

    chronos_soc_bus u_soc_bus (
        .clk_i        ( clk_i        ), .rst_ni       ( rst_sync_n   ),
        .cpu_daddr_i  ( cpu_daddr    ), .cpu_dwdata_i ( cpu_dwdata   ),
        .cpu_dbe_i    ( cpu_dbe      ), .cpu_dwe_i    ( cpu_dwe      ),
        .cpu_dreq_i   ( cpu_dreq     ), .cpu_drdata_o ( cpu_drdata   ),
        .cpu_dvalid_o ( cpu_dvalid   ),
        .rom_addr_o   ( rom_daddr    ), .rom_req_o    ( rom_dreq     ),
        .rom_rdata_i  ( rom_drdata   ), .rom_valid_i  ( rom_dvalid   ),
        .ram_addr_o   ( ram_addr     ), .ram_wdata_o  ( ram_wdata    ),
        .ram_be_o     ( ram_be       ), .ram_we_o     ( ram_we       ),
        .ram_req_o    ( ram_req      ), .ram_rdata_i  ( ram_rdata    ),
        .ram_valid_i  ( ram_valid    ),
        .dma_addr_o   ( dma_addr     ), .dma_wdata_o  ( dma_wdata    ),
        .dma_we_o     ( dma_we       ), .dma_req_o    ( dma_req      ),
        .dma_rdata_i  ( dma_rdata    ), .dma_valid_i  ( dma_valid    ),
        .hwpe_addr_o  ( hwpe_addr    ), .hwpe_wdata_o ( hwpe_wdata   ),
        .hwpe_we_o    ( hwpe_we      ), .hwpe_req_o   ( hwpe_req     ),
        .hwpe_rdata_i ( hwpe_rdata_q ), .hwpe_valid_i ( hwpe_valid_q ),
        .uart_addr_o  ( uart_addr    ), .uart_wdata_o ( uart_wdata   ),
        .uart_we_o    ( uart_we      ), .uart_req_o   ( uart_req     ),
        .uart_rdata_i ( uart_rdata   ), .uart_valid_i ( uart_valid   )
    );

    // 5. 4 KB Dual-Port Boot ROM
    (* ram_style = "block" *)
    logic [31:0] boot_rom [0:1023];

    initial begin
        if (BOOT_HEX != "") $readmemh(BOOT_HEX, boot_rom);
        else for (int i = 0; i < 1024; i++) boot_rom[i] = 32'h0000_0013;
    end

    always_ff @(posedge clk_i) begin
        if (rom_load_we_i) boot_rom[rom_load_addr_i] <= rom_load_data_i;
        cpu_idata  <= boot_rom[cpu_iaddr[11:2]];
        rom_drdata <= boot_rom[rom_daddr[11:2]];
    end

    always_ff @(posedge clk_i or negedge rst_sync_n) begin
        if (!rst_sync_n) begin cpu_ivalid <= 1'b0; rom_dvalid <= 1'b0; end
        else             begin cpu_ivalid <= cpu_ireq; rom_dvalid <= rom_dreq; end
    end

    // 6. 4 KB True Dual-Port Data SRAM (Port A: CPU, Port B: Autonomous DMA!)
    (* ram_style = "block" *)
    logic [31:0] data_sram [0:1023];
    logic        dma_sram_re;
    logic [9:0]  dma_sram_waddr;
    logic [31:0] dma_sram_rdata;

    always_ff @(posedge clk_i) begin
        if (ram_req && ram_we) begin
            if (ram_be[0]) data_sram[ram_addr[11:2]][7:0]   <= ram_wdata[7:0];
            if (ram_be[1]) data_sram[ram_addr[11:2]][15:8]  <= ram_wdata[15:8];
            if (ram_be[2]) data_sram[ram_addr[11:2]][23:16] <= ram_wdata[23:16];
            if (ram_be[3]) data_sram[ram_addr[11:2]][31:24] <= ram_wdata[31:24];
        end
        ram_rdata <= data_sram[ram_addr[11:2]];
    end

    always_ff @(posedge clk_i) begin
        if (dma_sram_re) dma_sram_rdata <= data_sram[dma_sram_waddr];
    end

    always_ff @(posedge clk_i or negedge rst_sync_n) begin
        if (!rst_sync_n) ram_valid <= 1'b0;
        else             ram_valid <= ram_req;
    end

    // 7. Autonomous Burst Stream DMA (0x3000_0000) & Full-Duplex UART (0x5000_0000)
    logic              dma_stream_we, uart_auto_we;
    logic signed [7:0] dma_stream_data, uart_auto_data;

    chronos_dma_ctrl u_dma (
        .clk_i             ( clk_i           ),
        .rst_ni            ( rst_sync_n      ),
        .req_i             ( dma_req         ),
        .we_i              ( dma_we          ),
        .addr_i            ( dma_addr        ),
        .wdata_i           ( dma_wdata       ),
        .rdata_o           ( dma_rdata       ),
        .valid_o           ( dma_valid       ),
        .sram_re_o         ( dma_sram_re     ),
        .sram_waddr_o      ( dma_sram_waddr  ),
        .sram_rdata_i      ( dma_sram_rdata  ),
        .dma_stream_we_o   ( dma_stream_we   ),
        .dma_stream_data_o ( dma_stream_data ),
        .dma_done_irq_o    ( dma_done_irq    )
    );

    chronos_uart_duplex #(
        .CLK_FREQ_HZ ( CLK_FREQ_HZ ), .BAUD_RATE ( BAUD_RATE )
    ) u_uart (
        .clk_i              ( clk_i          ), .rst_ni             ( rst_sync_n     ),
        .req_i              ( uart_req       ), .we_i               ( uart_we        ),
        .addr_i             ( uart_addr      ), .wdata_i            ( uart_wdata     ),
        .rdata_o            ( uart_rdata     ), .valid_o            ( uart_valid     ),
        .auto_stream_we_o   ( uart_auto_we   ), .auto_stream_data_o ( uart_auto_data ),
        .uart_rxd_i         ( uart_rxd_i     ), .uart_txd_o         ( uart_txd_o     )
    );

    // 8. CHRONOS 4+1 Concurrent Zero-DSP Log-SSM HWPE (0x4000_0000)
    //    3-way prioritized streaming input mux: DMA -> UART Auto-Stream -> CPU MMIO
    logic              cpu_stream_we, mux_stream_we;
    logic signed [7:0] mux_stream_data;
    logic              ring_full_unused;

    assign cpu_stream_we   = hwpe_req && hwpe_we && (hwpe_addr == 8'h40);
    assign mux_stream_we   = dma_stream_we | uart_auto_we | cpu_stream_we;
    assign mux_stream_data = dma_stream_we ? dma_stream_data :
                             (uart_auto_we ? uart_auto_data  : $signed(hwpe_wdata[7:0]));

    always_ff @(posedge clk_i or negedge rst_sync_n) begin
        if (!rst_sync_n) begin hwpe_valid_q <= 1'b0; hwpe_rdata_q <= 32'd0; end
        else             begin hwpe_valid_q <= hwpe_req; hwpe_rdata_q <= hwpe_rdata_comb; end
    end

    chronos_ssm_hwpe_top u_chronos_hwpe (
        .clk_i                ( clk_i                            ),
        .rst_ni               ( rst_sync_n                       ),
        .test_en_i            ( test_en_i                        ),
        .csr_req_i            ( hwpe_req && (hwpe_addr != 8'h40) ),
        .csr_we_i             ( hwpe_we                          ),
        .csr_addr_i           ( hwpe_addr                        ),
        .csr_wdata_i          ( hwpe_wdata                       ),
        .csr_rdata_o          ( hwpe_rdata_comb                  ),
        .irq_o                ( hwpe_irq_o                       ),
        .stream_we_i          ( mux_stream_we                    ),
        .stream_data_i        ( mux_stream_data                  ),
        .ring_full_o          ( ring_full_unused                 ),
        .w_load_we_i          ( w_load_we_i                      ),
        .w_load_addr_i        ( w_load_addr_i                    ),
        .w_load_data_i        ( w_load_data_i                    ),
        .bias_load_we_i       ( bias_load_we_i                   ),
        .bias_load_addr_i     ( bias_load_addr_i                 ),
        .bias_load_data_i     ( bias_load_data_i                 ),
        .a_load_we_i          ( a_load_we_i                      ),
        .a_load_addr_i        ( a_load_addr_i                    ),
        .a_load_data_i        ( a_load_data_i                    ),
        .d_load_we_i          ( d_load_we_i                      ),
        .d_load_addr_i        ( d_load_addr_i                    ),
        .d_load_data_i        ( d_load_data_i                    ),
        .conv_w_load_we_i     ( conv_w_load_we_i                 ),
        .conv_w_load_addr_i   ( conv_w_load_addr_i               ),
        .conv_w_load_data_i   ( conv_w_load_data_i               ),
        .pmu_energy_proxy_i   ( pmu_energy_proxy                 ),
        .pmu_cpu_sleep_cyc_i  ( pmu_cpu_sleep_cyc                ),
        .pmu_overlap_cyc_i    ( pmu_overlap_cyc                  ),
        .pmu_cnt_clr_o        ( pmu_cnt_clr                      ),
        .pmu_eng_a_active_o   ( pmu_eng_a_act                    ),
        .pmu_eng_b_active_o   ( pmu_eng_b_act                    ),
        .pmu_sram_rw_active_o ( pmu_sram_act                     ),
        .busy_o               ( hwpe_busy_o                      ),
        .done_o               ( hwpe_done_o                      ),
        .pred_class_o         ( pred_class_o                     )
    );

    wire _unused_soc = &{1'b0, cpu_iaddr[31:12], cpu_iaddr[1:0], rom_daddr[1:0],
                         ram_addr[1:0], irq_ack_unused, ring_full_unused};

endmodule : chronos_soc_top
