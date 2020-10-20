/* R-type */
`define OP_REG 7'b0110011

/* I-type */
`define OP_IMM 7'b0010011
`define OP_LOAD 7'b0000011
`define OP_JALR 7'b1100111

/* S-type */
`define OP_STORE 7'b0100011

/* B-type */
`define OP_BRANCH 7'b1100011

/* U-type */
`define OP_LUI 7'b0110111
`define OP_AUIPC 7'b0010111

/* J-type */
`define OP_JUMP 7'b1101111

/* Other instructions */
`define OP_FENCE 7'b0001111
`define OP_ECALL 7'b1110011
`define OP_NOOP 7'b0000000

/* This decoder extracts fixed length fields from a 32-bit instruction. It also
   generates an immediate based on the opcode (except R-type has none). */
module instruction_decoder
(
    input wire [31:0] iword,
    output wire [6:0] opcode,
    output wire [4:0] rd,
    output wire [4:0] rs1,
    output wire [4:0] rs2,
    output wire [2:0] funct3,
    output wire [6:0] funct7,
    output reg [31:0] imm /* Not a hardware register! We just assign to it in an always block. */
);
    /* fixed length fields */
    assign opcode = iword[6:0];
    assign rd = iword[11:7];
    assign funct3 = iword[14:12];
    assign rs1 = iword[19:15];
    assign rs2 = iword[24:20];
    assign funct7 = iword[31:25];

    /* immediate generation based on opcode */
    always @(*) begin
        /* I-type(0010011, 0000011, 1100111): imm[11:0] = iword[31:20]
           S-type(0100011): imm[11:5] = iword[31:25], imm[4:0] = iword[11:7]
           B-type(1100011): imm[12] = iword[31], imm[10:5] = iword[30:25], imm[4:1] = iword[11:8], imm[11] = iword[7], imm[0] = 0
           U-type(0110111, 0010111): imm[31:12] = iword[31:12], imm[11:0] = 0
           J-type(1101111): imm[20] = iword[31], imm[10:1] = iword[30:21], imm[11] = iword[20], imm[19:12] = iword[19:12], imm[0] = 0 */
        case (opcode)
            `OP_IMM: begin
                case (funct3)
                  3'b001, 3'b101: imm = iword[24:20]; /* SLLI, SRLI, SRAI only use 5 bits */
                  default: imm = $signed(iword[31:20]);
                endcase
            end
            `OP_LOAD, `OP_JALR: imm = $signed(iword[31:20]);
            `OP_STORE: imm = $signed({iword[31:25], iword[11:7]});
            `OP_BRANCH: imm = $signed({iword[31], iword[7], iword[30:25], iword[11:8], 1'b0});
            `OP_LUI, `OP_AUIPC: imm = {iword[31:12], {12{1'b0}}};
            `OP_JUMP: imm = $signed({iword[31], iword[19:12], iword[20], iword[30:21], 1'b0});
            default: imm = 32'bX;
        endcase
    end
endmodule
