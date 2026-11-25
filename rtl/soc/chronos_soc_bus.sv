// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/soc/chronos_soc_bus.sv
// Desc   : 5-Slave Registered-Response Crossbar Interconnect for CHRONOS SoC.
//          Memory Map:
//            - 0x0000_0000 : Boot ROM (4 KB)
//            - 0x2000_0000 : Data SRAM (4 KB)
//            - 0x3000_0000 : Autonomous Burst Stream DMA (`chronos_dma_ctrl`)
//            - 0x4000_0000 : CHRONOS 4+1 Mamba-SSM HWPE (CSRs + Stream Port)
//            - 0x5000_0000 : Full-Duplex UART Rx/Tx + Auto-Stream DMA
// ============================================================================

`timescale 1ns / 1ps

module chronos_soc_bus (
    input  logic        clk_i,
    input  logic        rst_ni,

    // CPU Data Master
    input  logic [31:0] cpu_daddr_i,
    input  logic [31:0] cpu_dwdata_i,
    input  logic [3:0]  cpu_dbe_i,
    input  logic        cpu_dwe_i,
    input  logic        cpu_dreq_i,
    output logic [31:0] cpu_drdata_o,
    output logic        cpu_dvalid_o,

    // Slave 0: Boot ROM (0x0000_0000)
    output logic [11:0] rom_addr_o,
    output logic        rom_req_o,
    input  logic [31:0] rom_rdata_i,
    input  logic        rom_valid_i,

    // Slave 1: Data SRAM (0x2000_0000)
    output logic [11:0] ram_addr_o,
    output logic [31:0] ram_wdata_o,
    output logic [3:0]  ram_be_o,
    output logic        ram_we_o,
    output logic        ram_req_o,
    input  logic [31:0] ram_rdata_i,
    input  logic        ram_valid_i,

    // Slave 2: Autonomous Stream DMA (0x3000_0000)
    output logic [3:0]  dma_addr_o,
    output logic [31:0] dma_wdata_o,
    output logic        dma_we_o,
    output logic        dma_req_o,
    input  logic [31:0] dma_rdata_i,
    input  logic        dma_valid_i,

    // Slave 3: CHRONOS Mamba-SSM HWPE (0x4000_0000)
    output logic [7:0]  hwpe_addr_o,
    output logic [31:0] hwpe_wdata_o,
    output logic        hwpe_we_o,
    output logic        hwpe_req_o,
    input  logic [31:0] hwpe_rdata_i,
    input  logic        hwpe_valid_i,

    // Slave 4: Full-Duplex UART (0x5000_0000)
    output logic [3:0]  uart_addr_o,
    output logic [31:0] uart_wdata_o,
    output logic        uart_we_o,
    output logic        uart_req_o,
    input  logic [31:0] uart_rdata_i,
    input  logic        uart_valid_i
);

    localparam logic [2:0] SEL_ROM  = 3'd0;
    localparam logic [2:0] SEL_RAM  = 3'd1;
    localparam logic [2:0] SEL_DMA  = 3'd2;
    localparam logic [2:0] SEL_HWPE = 3'd3;
    localparam logic [2:0] SEL_UART = 3'd4;

    logic [2:0] target_sel, target_sel_q;
    wire _unused_bus = &{1'b0, cpu_daddr_i[27:12]};

    always_comb begin
        case (cpu_daddr_i[31:28])
            4'h0:    target_sel = SEL_ROM;
            4'h2:    target_sel = SEL_RAM;
            4'h3:    target_sel = SEL_DMA;
            4'h4:    target_sel = SEL_HWPE;
            4'h5:    target_sel = SEL_UART;
            default: target_sel = SEL_ROM;
        endcase
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni)        target_sel_q <= SEL_ROM;
        else if (cpu_dreq_i) target_sel_q <= target_sel;
    end

    assign rom_addr_o   = cpu_daddr_i[11:0];
    assign rom_req_o    = cpu_dreq_i && (target_sel == SEL_ROM);

    assign ram_addr_o   = cpu_daddr_i[11:0];
    assign ram_wdata_o  = cpu_dwdata_i;
    assign ram_be_o     = cpu_dbe_i;
    assign ram_we_o     = cpu_dwe_i;
    assign ram_req_o    = cpu_dreq_i && (target_sel == SEL_RAM);

    assign dma_addr_o   = cpu_daddr_i[3:0];
    assign dma_wdata_o  = cpu_dwdata_i;
    assign dma_we_o     = cpu_dwe_i;
    assign dma_req_o    = cpu_dreq_i && (target_sel == SEL_DMA);

    assign hwpe_addr_o  = cpu_daddr_i[7:0];
    assign hwpe_wdata_o = cpu_dwdata_i;
    assign hwpe_we_o    = cpu_dwe_i;
    assign hwpe_req_o   = cpu_dreq_i && (target_sel == SEL_HWPE);

    assign uart_addr_o  = cpu_daddr_i[3:0];
    assign uart_wdata_o = cpu_dwdata_i;
    assign uart_we_o    = cpu_dwe_i;
    assign uart_req_o   = cpu_dreq_i && (target_sel == SEL_UART);

    always_comb begin
        case (target_sel_q)
            SEL_ROM:  begin cpu_drdata_o = rom_rdata_i;  cpu_dvalid_o = rom_valid_i;  end
            SEL_RAM:  begin cpu_drdata_o = ram_rdata_i;  cpu_dvalid_o = ram_valid_i;  end
            SEL_DMA:  begin cpu_drdata_o = dma_rdata_i;  cpu_dvalid_o = dma_valid_i;  end
            SEL_HWPE: begin cpu_drdata_o = hwpe_rdata_i; cpu_dvalid_o = hwpe_valid_i; end
            SEL_UART: begin cpu_drdata_o = uart_rdata_i; cpu_dvalid_o = uart_valid_i; end
            default:  begin cpu_drdata_o = 32'd0;        cpu_dvalid_o = 1'b0;         end
        endcase
    end

endmodule : chronos_soc_bus
