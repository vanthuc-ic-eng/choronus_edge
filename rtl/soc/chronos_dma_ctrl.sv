// ============================================================================
// Project: CHRONOS — Beyond-Q1 Zero-DSP Logarithmic Selective State-Space SoC
// File   : rtl/soc/chronos_dma_ctrl.sv
// Desc   : ★ BEYOND-Q1 UPGRADE — Autonomous Zero-CPU Burst Stream DMA ★
//          Memory-Mapped at 0x3000_0000:
//            - 0x00: DMA_SRC_ADDR (byte address in 4 KB Data SRAM)
//            - 0x04: DMA_LEN      (number of INT8 samples to stream to Bank 0)
//            - 0x08: DMA_CTRL     ([0]=START, [1]=BUSY(R), [2]=DONE(R/W1C))
//          Streams 1 INT8 sample/cycle from Data SRAM Port B directly into
//          CHRONOS Bank 0 Ring Buffer with ZERO CPU copy overhead!
// ============================================================================

`timescale 1ns / 1ps

module chronos_dma_ctrl (
    input  logic        clk_i,
    input  logic        rst_ni,

    // Memory-Mapped Bus Slave Port (0x3000_0000)
    input  logic        req_i,
    input  logic        we_i,
    input  logic [3:0]  addr_i,
    input  logic [31:0] wdata_i,
    output logic [31:0] rdata_o,
    output logic        valid_o,

    // Dedicated Read Port to Data SRAM (Port B - Zero CPU Stall!)
    output logic        sram_re_o,
    output logic [9:0]  sram_waddr_o,
    input  logic [31:0] sram_rdata_i,

    // Direct Push Port to CHRONOS Bank 0 Ring Buffer
    output logic        dma_stream_we_o,
    output logic signed [7:0] dma_stream_data_o,
    output logic        dma_done_irq_o
);

    logic [11:0] src_addr_q;
    logic [11:0] cur_addr_q;
    logic [11:0] rem_len_q;
    logic        busy_q;
    logic        done_q;
    logic        rd_valid_d1_q;
    logic [1:0]  byte_sel_d1_q;

    wire _unused_dma = &{1'b0, addr_i[1:0], wdata_i[31:12]};

    // Select exact byte from 32-bit SRAM word
    logic [7:0] selected_byte;
    always_comb begin
        case (byte_sel_d1_q)
            2'b00: selected_byte = sram_rdata_i[7:0];
            2'b01: selected_byte = sram_rdata_i[15:8];
            2'b10: selected_byte = sram_rdata_i[23:16];
            2'b11: selected_byte = sram_rdata_i[31:24];
        endcase
    end

    assign sram_re_o         = busy_q && (rem_len_q != 12'd0);
    assign sram_waddr_o      = cur_addr_q[11:2];
    assign dma_stream_we_o   = rd_valid_d1_q;
    assign dma_stream_data_o = $signed(selected_byte);
    assign dma_done_irq_o    = done_q;

    // Bus Read/Write & Burst Streaming FSM
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            valid_o       <= 1'b0;
            rdata_o       <= 32'd0;
            src_addr_q    <= 12'd0;
            cur_addr_q    <= 12'd0;
            rem_len_q     <= 12'd0;
            busy_q        <= 1'b0;
            done_q        <= 1'b0;
            rd_valid_d1_q <= 1'b0;
            byte_sel_d1_q <= 2'b00;
        end else begin
            valid_o       <= req_i;
            rd_valid_d1_q <= 1'b0;

            if (req_i && !we_i) begin
                case (addr_i[3:2])
                    2'b00:   rdata_o <= {20'd0, src_addr_q};
                    2'b01:   rdata_o <= {20'd0, rem_len_q};
                    2'b10:   rdata_o <= {29'd0, done_q, busy_q, 1'b0};
                    default: rdata_o <= 32'd0;
                endcase
            end else begin
                rdata_o <= 32'd0;
            end

            // CSR Writes
            if (req_i && we_i) begin
                case (addr_i[3:2])
                    2'b00: src_addr_q <= wdata_i[11:0];
                    2'b01: rem_len_q  <= wdata_i[11:0];
                    2'b10: begin
                        if (wdata_i[2]) done_q <= 1'b0; // W1C
                        if (wdata_i[0] && (rem_len_q != 12'd0)) begin
                            cur_addr_q <= src_addr_q;
                            busy_q     <= 1'b1;
                            done_q     <= 1'b0;
                        end
                    end
                    default: ;
                endcase
            end

            // Active 1-byte/cycle Burst Transfer
            if (busy_q) begin
                if (rem_len_q != 12'd0) begin
                    rd_valid_d1_q <= 1'b1;
                    byte_sel_d1_q <= cur_addr_q[1:0];
                    cur_addr_q    <= cur_addr_q + 12'd1;
                    rem_len_q     <= rem_len_q  - 12'd1;
                end else begin
                    busy_q <= 1'b0;
                    done_q <= 1'b1;
                end
            end
        end
    end

endmodule : chronos_dma_ctrl
