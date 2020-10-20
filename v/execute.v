`include "decoder.v"
`include "memory.v"
`include "registers.v"

`define ALU_ADD 4'b0000
`define ALU_SUB 4'b0001
`define ALU_SLL 4'b0010
`define ALU_SLT 4'b0011
`define ALU_SLTU 4'b0100
`define ALU_XOR 4'b0101
`define ALU_SRL 4'b0110
`define ALU_SRA 4'b0111
`define ALU_OR 4'b1000
`define ALU_AND 4'b1001
`define ALU_LUI 4'b1010
`define ALU_JALR 4'b1011

`define PCSEL_4 1'b0
`define PCSEL_ALU 1'b1

`define ASEL_REG 1'b0
`define ASEL_PC 1'b1

`define BSEL_REG 1'b0
`define BSEL_IMM 1'b1

`define MEM_READ 1'b1
`define MEM_WRITE 1'b0

`define WBSEL_PC4 2'b00
`define WBSEL_ALU 2'b01
`define WBSEL_MEM 2'b10

/* Outputs (signed or unsigned) equality and less than comparisons. */
module branch_compare
(
    input wire [31:0] data_rs1,
    input wire [31:0] data_rs2,
    input wire BrUn,
    output wire BrEq,
    output wire BrLt
);
    assign BrEq = data_rs1 == data_rs2;
    assign BrLt = BrUn ? data_rs1 < data_rs2 : $signed(data_rs1) < $signed(data_rs2);
endmodule

/* Performs arithmetic and logic based on control signals. */
module alu
(
    input wire [3:0] ALUSel,
    input wire [31:0] a,
    input wire [31:0] b,
    output reg [31:0] out
);
    always @(*) begin
        case (ALUSel)
            `ALU_ADD: out = a + b;
            `ALU_JALR: out = (a + b) & 32'hffff_fffe;
            `ALU_SUB: out = a - b;
            `ALU_SLL: out = a << b[4:0];
            `ALU_SLT: out = $signed(a) < $signed(b) ? 1'b1 : 1'b0;
            `ALU_SLTU: out = a < b ? 1'b1 : 1'b0;
            `ALU_XOR: out = a ^ b;
            `ALU_SRL: out = a >> b[4:0];
            `ALU_SRA: out = $signed(a) >>> b[4:0];
            `ALU_OR: out = a | b;
            `ALU_AND: out = a & b;
            `ALU_LUI: out = b;
            default: out = 32'bX;
        endcase
    end
endmodule

module control
(
    input wire [6:0] opcode,
    input wire [2:0] funct3,
    input wire [6:0] funct7,
    output reg PCSel,
    output reg RegWEn,
    output reg BrUn, /* branch_compare input */
    input wire BrEq, /* branch_compare output */
    input wire BrLt, /* branch_compare output */
    output reg ASel,
    output reg BSel,
    output reg [3:0] ALUSel,
    output reg MemRW,
    output reg [1:0] WBSel,
    output reg [1:0] mem_width,
    output reg mem_signed_read
);
    always @(*) begin
        case (opcode)
            `OP_REG: begin
                PCSel=`PCSEL_4; ASel=`ASEL_REG; BSel=`BSEL_REG; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=1; WBSel=`WBSEL_ALU;
                case (funct3)
                    3'b000:
                        case (funct7[5])
                            1'b0: begin ALUSel=`ALU_ADD; end
                            1'b1: begin ALUSel=`ALU_SUB; end
                        endcase
                    3'b001: ALUSel=`ALU_SLL;
                    3'b010: ALUSel=`ALU_SLT;
                    3'b011: ALUSel=`ALU_SLTU;
                    3'b100: ALUSel=`ALU_XOR;
                    3'b101:
                        case (funct7[5])
                            1'b0: ALUSel=`ALU_SRL;
                            1'b1: ALUSel=`ALU_SRA;
                        endcase
                    3'b110: ALUSel=`ALU_OR;
                    3'b111: ALUSel=`ALU_AND;
                endcase
            end
            `OP_IMM: begin
                PCSel=`PCSEL_4; ASel=`ASEL_REG; BSel=`BSEL_IMM; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=1; WBSel=`WBSEL_ALU;
                case (funct3)
                    3'b000: ALUSel=`ALU_ADD;
                    3'b010: ALUSel=`ALU_SLT;
                    3'b011: ALUSel=`ALU_SLTU;
                    3'b100: ALUSel=`ALU_XOR;
                    3'b110: ALUSel=`ALU_OR;
                    3'b111: ALUSel=`ALU_AND;
                    3'b001: ALUSel=`ALU_SLL;
                    3'b101:
                        case (funct7[5])
                            1'b0:ALUSel=`ALU_SRL;
                            1'b1:ALUSel=`ALU_SRA;
                        endcase
                endcase
            end
            `OP_JALR: begin
                PCSel=`PCSEL_ALU; ASel=`ASEL_REG; BSel=`BSEL_IMM; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=1; WBSel=`WBSEL_PC4;
                ALUSel=`ALU_JALR;
            end
            `OP_LOAD: begin
                PCSel=`PCSEL_4; ASel=`ASEL_REG; BSel=`BSEL_IMM; MemRW=`MEM_READ; RegWEn=1; WBSel=`WBSEL_MEM;
                case (funct3)
                    3'b000: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_B; mem_signed_read=1; end
                    3'b001: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_H; mem_signed_read=1; end
                    3'b010: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_W; mem_signed_read=0; end
                    3'b100: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_B; mem_signed_read=0; end
                    3'b101: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_H; mem_signed_read=0; end
                    default: ALUSel=4'bX;
                endcase
            end
            `OP_STORE: begin
                PCSel=`PCSEL_4; ASel=`ASEL_REG; BSel=`BSEL_IMM; MemRW=`MEM_WRITE; RegWEn=0; WBSel=2'bX;
                case (funct3)
                    3'b000: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_B; end
                    3'b001: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_H; end
                    3'b010: begin ALUSel=`ALU_ADD; mem_width=`MEM_WIDTH_W; end
                    /* Don't modify memory if the instruction is unknown */
                    default: begin ALUSel=4'bX; MemRW=`MEM_READ; end
                endcase
                end
            `OP_BRANCH: begin
                ASel=`ASEL_PC; BSel=`BSEL_IMM; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=0; WBSel=2'bX;
                case (funct3)
                    3'b000: begin ALUSel=`ALU_ADD; BrUn=0; PCSel=BrEq ? `PCSEL_ALU : `PCSEL_4; end
                    3'b001: begin ALUSel=`ALU_ADD; BrUn=0; PCSel=BrEq ? `PCSEL_4 : `PCSEL_ALU; end
                    3'b100: begin ALUSel=`ALU_ADD; BrUn=0; PCSel=BrLt ? `PCSEL_ALU : `PCSEL_4; end
                    3'b101: begin ALUSel=`ALU_ADD; BrUn=0; PCSel=BrLt ? `PCSEL_4 : `PCSEL_ALU; end
                    3'b110: begin ALUSel=`ALU_ADD; BrUn=1; PCSel=BrLt ? `PCSEL_ALU : `PCSEL_4; end
                    3'b111: begin ALUSel=`ALU_ADD; BrUn=1; PCSel=BrLt ? `PCSEL_4 : `PCSEL_ALU; end
                    default: begin ALUSel=4'bX; end
                endcase
            end
            `OP_JUMP: begin
                ALUSel=`ALU_ADD;
                PCSel=`PCSEL_ALU; ASel=`ASEL_PC; BSel=`BSEL_IMM; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=1; WBSel=`WBSEL_PC4;
             end
            `OP_LUI: begin
                ALUSel=`ALU_LUI;
                PCSel=`PCSEL_4; ASel=1'bX; BSel=`BSEL_IMM; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=1; WBSel=`WBSEL_ALU;
            end
            `OP_AUIPC: begin
                ALUSel=`ALU_ADD;
                PCSel=`PCSEL_4; ASel=`ASEL_PC; BSel=`BSEL_IMM; MemRW=`MEM_READ; mem_width=2'bX; RegWEn=1; WBSel=`WBSEL_ALU;
            end
            /* After branching to no-op instructions, increment PC by 4 as normal */
            `OP_FENCE: begin PCSel=`PCSEL_4; MemRW=`MEM_READ; RegWEn=0; end
            `OP_ECALL: begin PCSel=`PCSEL_4; MemRW=`MEM_READ; RegWEn=0; end
            /* On no-op, do not write to either register file or data memory */
            `OP_NOOP: begin PCSel=`PCSEL_4; MemRW=`MEM_READ; RegWEn=0; end
            default: begin
                $display("[EXECUTE] Unknown opcode encountered: %h", opcode);
                $finish;
            end
        endcase
    end
endmodule
