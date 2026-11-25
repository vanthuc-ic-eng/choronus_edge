// ============================================================================
// Project: CHRONOS — Zero-DSP Logarithmic Selective State-Space Edge AI SoC
// File   : rtl/soc/chronos_riscv_rv32i.sv
// Desc   : Ultra-Compact 100 MHz RISC-V RV32I Host Control Core with:
//            - Decode Register Latching (cuts 32:1 regfile mux from ALU path)
//            - Shared 33-bit Adder/Subtractor & Bit-Reversal Barrel Shifter
//            - ★ Hardware Event-Driven WFI Sleep/Wakeup (`32'h1050_0073`) ★
//              Halts instruction fetch & datapath while CHRONOS HWPE runs,
//              waking in 1 cycle on Anomaly IRQ or HWPE Done!
//            - 100% Zero-DSP & Yosys/Verilator/Vivado/Sky130 Clean
// ============================================================================

`timescale 1ns / 1ps

(* use_dsp = "no" *)
module chronos_riscv_rv32i #(
    parameter logic [31:0] BOOT_ADDR = 32'h0000_0000
)(
    input  logic        clk_i,
    input  logic        rst_ni,

    // Instruction Fetch Port (Boot ROM)
    output logic [31:0] instr_addr_o,
    output logic        instr_req_o,
    input  logic [31:0] instr_data_i,
    input  logic        instr_valid_i,

    // Data Load/Store Port (SoC Crossbar)
    output logic [31:0] data_addr_o,
    output logic [31:0] data_wdata_o,
    output logic [3:0]  data_be_o,
    output logic        data_we_o,
    output logic        data_req_o,
    input  logic [31:0] data_rdata_i,
    input  logic        data_valid_i,

    // Hardware Event-Driven Wakeup / Interrupt Interface
    input  logic        wakeup_irq_i,    // Wakes CPU from WFI sleep (hwpe_irq | hwpe_done)
    output logic        cpu_sleeping_o,  // High while halted in WFI sleep
    output logic        irq_ack_o
);

    localparam logic [2:0] STATE_FETCH    = 3'd0;
    localparam logic [2:0] STATE_DECODE   = 3'd1;
    localparam logic [2:0] STATE_EXECUTE  = 3'd2;
    localparam logic [2:0] STATE_MEM_WAIT = 3'd3;
    localparam logic [2:0] STATE_SLEEP    = 3'd4; // WFI Low-Power Event Wait

    logic [2:0]  state_q, state_next;
    logic [31:0] pc_q, pc_next;
    logic [31:0] instr_q;

    (* ram_style = "distributed" *)
    logic [31:0] regfile [0:31];

    initial begin
        for (int i = 0; i < 32; i++) begin
            regfile[i] = 32'd0;
        end
    end

    // Instruction Decode Fields
    logic [6:0]  opcode;
    logic [4:0]  rd, rs1, rs2;
    logic [2:0]  funct3;
    logic [6:0]  funct7;
    logic        is_wfi;

    assign opcode = instr_q[6:0];
    assign rd     = instr_q[11:7];
    assign funct3 = instr_q[14:12];
    assign rs1    = instr_q[19:15];
    assign rs2    = instr_q[24:20];
    assign funct7 = instr_q[31:25];
    assign is_wfi = (instr_q == 32'h1050_0073); // Standard RV32I SYSTEM WFI

    // Immediates
    logic [31:0] imm_i, imm_s, imm_b, imm_u, imm_j;
    assign imm_i = {{20{instr_q[31]}}, instr_q[31:20]};
    assign imm_s = {{20{instr_q[31]}}, instr_q[31:25], instr_q[11:7]};
    assign imm_b = {{19{instr_q[31]}}, instr_q[31], instr_q[7], instr_q[30:25], instr_q[11:8], 1'b0};
    assign imm_u = {instr_q[31:12], 12'h000};
    assign imm_j = {{11{instr_q[31]}}, instr_q[31], instr_q[19:12], instr_q[20], instr_q[30:21], 1'b0};

    // Latched Register Operands (Decode Register Latching for >=100 MHz timing)
    logic [31:0] rs1_data_q, rs2_data_q;

    // Unified Area-Optimized ALU
    logic        alu_active;
    logic [31:0] alu_op_a, alu_op_b, alu_result;
    logic [4:0]  shift_amt;

    assign alu_active = (state_q == STATE_EXECUTE);
    assign shift_amt  = alu_op_b[4:0];

    always_comb begin
        if (!alu_active) begin
            alu_op_a = 32'd0;
            alu_op_b = 32'd0;
        end else begin
            case (opcode)
                7'b0110011,
                7'b1100011: begin alu_op_a = rs1_data_q; alu_op_b = rs2_data_q; end
                7'b0010011: begin alu_op_a = rs1_data_q; alu_op_b = imm_i;      end
                7'b0000011: begin alu_op_a = rs1_data_q; alu_op_b = imm_i;      end
                7'b0100011: begin alu_op_a = rs1_data_q; alu_op_b = imm_s;      end
                7'b0110111: begin alu_op_a = 32'd0;      alu_op_b = imm_u;      end
                7'b0010111: begin alu_op_a = pc_q;       alu_op_b = imm_u;      end
                default:    begin alu_op_a = rs1_data_q; alu_op_b = imm_i;      end
            endcase
        end
    end

    // Shared 33-bit Adder / Subtractor / Comparator
    logic        is_mem_or_upper, is_sub_or_cmp;
    logic [32:0] add_sub_full;
    logic        cmp_eq, cmp_ult, cmp_slt;

    assign is_mem_or_upper = (opcode == 7'b0000011) || (opcode == 7'b0100011) ||
                             (opcode == 7'b0110111) || (opcode == 7'b0010111);
    assign is_sub_or_cmp   = !is_mem_or_upper && (
                             (opcode == 7'b1100011) ||
                             (funct3 == 3'b010) || (funct3 == 3'b011) ||
                             ((opcode == 7'b0110011) && (funct3 == 3'b000) && funct7[5]));
    assign add_sub_full    = {1'b0, alu_op_a}
                           + {1'b0, (is_sub_or_cmp ? ~alu_op_b : alu_op_b)}
                           + {32'd0, is_sub_or_cmp};
    assign cmp_eq          = (alu_op_a == alu_op_b);
    assign cmp_ult         = ~add_sub_full[32];
    assign cmp_slt         = (alu_op_a[31] != alu_op_b[31]) ? alu_op_a[31] : add_sub_full[31];

    // Shared 32-bit Bit-Reversal Barrel Shifter
    function automatic logic [31:0] bit_rev32(input logic [31:0] v);
        for (int b = 0; b < 32; b++) bit_rev32[b] = v[31 - b];
    endfunction

    logic        is_sll, is_sra;
    logic [31:0] shift_in, shift_out;
    logic [32:0] shift_ext, shift_right;

    assign is_sll      = (funct3 == 3'b001);
    assign is_sra      = (funct3 == 3'b101) && funct7[5];
    assign shift_in    = is_sll ? bit_rev32(alu_op_a) : alu_op_a;
    assign shift_ext   = {is_sra & alu_op_a[31], shift_in};
    assign shift_right = $unsigned($signed(shift_ext) >>> shift_amt);
    assign shift_out   = is_sll ? bit_rev32(shift_right[31:0]) : shift_right[31:0];

    wire _unused_cpu = &{1'b0, funct7[6], funct7[4:0], shift_right[32]};

    always_comb begin
        if (is_mem_or_upper) begin
            alu_result = add_sub_full[31:0];
        end else begin
            case (funct3)
                3'b000:  alu_result = add_sub_full[31:0];
                3'b001,
                3'b101:  alu_result = shift_out;
                3'b010:  alu_result = {31'd0, cmp_slt};
                3'b011:  alu_result = {31'd0, cmp_ult};
                3'b100:  alu_result = alu_op_a ^ alu_op_b;
                3'b110:  alu_result = alu_op_a | alu_op_b;
                3'b111:  alu_result = alu_op_a & alu_op_b;
                default: alu_result = 32'd0;
            endcase
        end
    end

    // Branch Evaluation
    logic branch_taken;
    always_comb begin
        case (funct3)
            3'b000:  branch_taken =  cmp_eq;
            3'b001:  branch_taken = !cmp_eq;
            3'b100:  branch_taken =  cmp_slt;
            3'b101:  branch_taken = !cmp_slt;
            3'b110:  branch_taken =  cmp_ult;
            3'b111:  branch_taken = !cmp_ult;
            default: branch_taken = 1'b0;
        endcase
    end

    // Shared PC Target Adder
    logic [31:0] pc_add_base, pc_add_offset, pc_target, pc_plus4;
    assign pc_add_base   = (opcode == 7'b1100111) ? rs1_data_q : pc_q;
    assign pc_add_offset = (opcode == 7'b1101111) ? imm_j :
                           (opcode == 7'b1100111) ? imm_i : imm_b;
    assign pc_target     = pc_add_base + pc_add_offset;
    assign pc_plus4      = pc_q + 32'd4;

    // Registered Data Memory Interface
    logic [31:0] data_addr_q, data_wdata_q;
    logic [3:0]  data_be_q, data_be_comb;
    logic        data_req_q, data_we_q;
    logic [1:0]  byte_offset_q;
    logic [31:0] data_wdata_comb, load_data_formatted;

    assign instr_addr_o   = pc_q;
    assign instr_req_o    = (state_q == STATE_FETCH);
    assign data_addr_o    = data_addr_q;
    assign data_wdata_o   = data_wdata_q;
    assign data_be_o      = data_be_q;
    assign data_req_o     = data_req_q && (state_q == STATE_MEM_WAIT);
    assign data_we_o      = data_we_q;
    assign cpu_sleeping_o = (state_q == STATE_SLEEP);

    always_comb begin
        case (funct3[1:0])
            2'b00: begin data_be_comb = 4'b0001 << alu_result[1:0]; data_wdata_comb = {4{rs2_data_q[7:0]}}; end
            2'b01: begin data_be_comb = alu_result[1] ? 4'b1100 : 4'b0011; data_wdata_comb = {2{rs2_data_q[15:0]}}; end
            default: begin data_be_comb = 4'b1111; data_wdata_comb = rs2_data_q; end
        endcase
    end

    always_comb begin
        case (funct3)
            3'b000: case (byte_offset_q)
                2'b00: load_data_formatted = {{24{data_rdata_i[7]}},  data_rdata_i[7:0]};
                2'b01: load_data_formatted = {{24{data_rdata_i[15]}}, data_rdata_i[15:8]};
                2'b10: load_data_formatted = {{24{data_rdata_i[23]}}, data_rdata_i[23:16]};
                2'b11: load_data_formatted = {{24{data_rdata_i[31]}}, data_rdata_i[31:24]};
            endcase
            3'b001: load_data_formatted = byte_offset_q[1] ? {{16{data_rdata_i[31]}}, data_rdata_i[31:16]}
                                                           : {{16{data_rdata_i[15]}}, data_rdata_i[15:0]};
            3'b010: load_data_formatted = data_rdata_i;
            3'b100: case (byte_offset_q)
                2'b00: load_data_formatted = {24'd0, data_rdata_i[7:0]};
                2'b01: load_data_formatted = {24'd0, data_rdata_i[15:8]};
                2'b10: load_data_formatted = {24'd0, data_rdata_i[23:16]};
                2'b11: load_data_formatted = {24'd0, data_rdata_i[31:24]};
            endcase
            3'b101: load_data_formatted = byte_offset_q[1] ? {16'd0, data_rdata_i[31:16]}
                                                           : {16'd0, data_rdata_i[15:0]};
            default: load_data_formatted = data_rdata_i;
        endcase
    end

    // Next-State & Writeback Control Logic
    logic        reg_write_en;
    logic [31:0] reg_write_data;

    always_comb begin
        state_next     = state_q;
        pc_next        = pc_q;
        reg_write_en   = 1'b0;
        reg_write_data = 32'd0;
        irq_ack_o      = 1'b0;

        case (state_q)
            STATE_FETCH: begin
                if (instr_valid_i) state_next = STATE_DECODE;
            end

            STATE_DECODE: begin
                state_next = STATE_EXECUTE;
            end

            STATE_EXECUTE: begin
                if (is_wfi) begin
                    // Enter Event-Driven Low-Power Sleep until HWPE IRQ/Done!
                    pc_next    = pc_plus4;
                    state_next = wakeup_irq_i ? STATE_FETCH : STATE_SLEEP;
                    irq_ack_o  = wakeup_irq_i;
                end else begin
                    case (opcode)
                        7'b0110011, 7'b0010011, 7'b0110111, 7'b0010111: begin
                            reg_write_en   = (rd != 5'd0);
                            reg_write_data = alu_result;
                            pc_next        = pc_plus4;
                            state_next     = STATE_FETCH;
                        end
                        7'b1101111: begin // JAL
                            reg_write_en   = (rd != 5'd0);
                            reg_write_data = pc_plus4;
                            pc_next        = pc_target;
                            state_next     = STATE_FETCH;
                        end
                        7'b1100111: begin // JALR
                            reg_write_en   = (rd != 5'd0);
                            reg_write_data = pc_plus4;
                            pc_next        = pc_target & ~32'd1;
                            state_next     = STATE_FETCH;
                        end
                        7'b1100011: begin // BRANCH
                            pc_next    = branch_taken ? pc_target : pc_plus4;
                            state_next = STATE_FETCH;
                        end
                        7'b0000011, 7'b0100011: begin // LOAD / STORE
                            state_next = STATE_MEM_WAIT;
                        end
                        default: begin
                            pc_next    = pc_plus4;
                            state_next = STATE_FETCH;
                        end
                    endcase
                end
            end

            STATE_MEM_WAIT: begin
                if (data_valid_i) begin
                    if (opcode == 7'b0000011) begin
                        reg_write_en   = (rd != 5'd0);
                        reg_write_data = load_data_formatted;
                    end
                    pc_next    = pc_plus4;
                    state_next = STATE_FETCH;
                end
            end

            STATE_SLEEP: begin
                if (wakeup_irq_i) begin
                    irq_ack_o  = 1'b1;
                    state_next = STATE_FETCH;
                end
            end

            default: state_next = STATE_FETCH;
        endcase
    end

    // Sequential State Update
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state_q       <= STATE_FETCH;
            pc_q          <= BOOT_ADDR;
            instr_q       <= 32'h0000_0013; // NOP
            rs1_data_q    <= 32'd0;
            rs2_data_q    <= 32'd0;
            byte_offset_q <= 2'b00;
            data_addr_q   <= 32'd0;
            data_wdata_q  <= 32'd0;
            data_be_q     <= 4'd0;
            data_req_q    <= 1'b0;
            data_we_q     <= 1'b0;
        end else begin
            state_q <= state_next;
            pc_q    <= pc_next;

            if (state_q == STATE_DECODE) begin
                rs1_data_q <= (rs1 == 5'd0) ? 32'd0 : regfile[rs1];
                rs2_data_q <= (rs2 == 5'd0) ? 32'd0 : regfile[rs2];
            end

            if (state_q == STATE_EXECUTE && !is_wfi) begin
                byte_offset_q <= alu_result[1:0];
                if ((opcode == 7'b0000011) || (opcode == 7'b0100011)) begin
                    data_addr_q  <= alu_result;
                    data_wdata_q <= data_wdata_comb;
                    data_be_q    <= data_be_comb;
                    data_req_q   <= 1'b1;
                    data_we_q    <= (opcode == 7'b0100011);
                end else begin
                    data_req_q   <= 1'b0;
                end
            end else begin
                data_req_q <= 1'b0;
            end

            if (state_q == STATE_FETCH && instr_valid_i) begin
                instr_q <= instr_data_i;
            end
        end
    end

    always_ff @(posedge clk_i) begin
        if (reg_write_en && (rd != 5'd0)) begin
            regfile[rd] <= reg_write_data;
        end
    end

endmodule : chronos_riscv_rv32i
