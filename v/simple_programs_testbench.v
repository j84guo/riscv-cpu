`include "execute.v"

module simple_programs_testbench;
    /* Clock */
    reg clock = 1;
    always begin
        #5 clock = ~clock;
    end

    /* Pipeline program counter */
    wire pc_f_d_stall;
    reg [31:0] pc_f = `BASE_ADDRESS, pc_d, pc_x, pc_m, pc_w;
    always @(posedge clock) begin
        if (pc_f_d_stall)
            pc_d <= pc_f;
        pc_x <= pc_d; pc_m <= pc_x; pc_w <= pc_m;
    end

    /* --------------
       1. FETCH STAGE
       -------------- */
    /* Instruction memory */
    wire [31:0] imem_data_out;
    memory imem
    (
        .clock(clock),
        .address(pc_f),
        .data_in(32'hxxxx_xxxx),
        .data_out(imem_data_out),
        .read_write(`MEM_READ),
        .width(`MEM_WIDTH_W),
        .signed_read(1'b0)
    );
    reg [31:0] inst_d = 32'h0000_0000;

    /* ---------------------------------
       2. DECODE AND REGISTER READ STAGE
       --------------------------------- */
    /* Decoder */
    wire [6:0] opcode; reg [6:0] opcode_x = 7'b0000000, opcode_m = 7'b0000000, opcode_w = 7'b0000000;
    wire [4:0] rd; reg [4:0] rd_x, rd_m, rd_w;
    wire [4:0] rs1; reg [4:0] rs1_x;
    wire [4:0] rs2; reg [4:0] rs2_x, rs2_m;
    wire [2:0] funct3; reg [2:0] funct3_x;
    wire [6:0] funct7; reg [6:0] funct7_x;
    wire [31:0] imm; reg [31:0] imm_x;
    instruction_decoder deco
    (
        .iword(inst_d),
        .opcode(opcode),
        .rd(rd),
        .rs1(rs1),
        .rs2(rs2),
        .funct3(funct3),
        .funct7(funct7),
        .imm(imm)
    );

    /* Register File */
    reg [31:0] data_rd_w;
    wire [31:0] data_rs1; reg [31:0] data_rs1_x;
    wire [31:0] data_rs2; reg [31:0] data_rs2_x, data_rs2_m;
    reg RegWEn_m, RegWEn_w;
    register_file regf
    (
        .clock(clock), /* in from clock */
        .addr_rs1(rs1), /* in from decoder */
        .addr_rs2(rs2), /* in from decoder */
        .addr_rd(rd_w), /* in from W-stage */
        .data_rd(data_rd_w), /* in from W-stage */
        .data_rs1(data_rs1),
        .data_rs2(data_rs2),
        .write_enable(RegWEn_w) /* in from W-stage */
    );

    /* Pipeline decoder and register file outputs */
    always @(posedge clock) begin
        rs1_x <= rs1;
        rs2_x <= rs2; rs2_m <= rs2_x;
        rd_x <= rd; rd_m <= rd_x; rd_w <= rd_m;
        funct3_x <= funct3;
        funct7_x <= funct7;
        imm_x <= imm;
        data_rs1_x <= data_rs1;
        data_rs2_x <= data_rs2; data_rs2_m <= data_rs2_mx_bypass ? alu_out_m : (data_rs2_wx_bypass ? data_rd_w : data_rs2_x);
    end

    /* ----------------
       3. EXECUTE STAGE
       ---------------- */
    /* Register alu output */
    reg [31:0] alu_out_m;

    /* Detect bypass conditions */
    wire data_rs1_mx_bypass, data_rs1_wx_bypass;
    wire data_rs2_mx_bypass, data_rs2_wx_bypass;
    wire data_rs2_wm_bypass;

    assign data_rs1_mx_bypass =
        /* X-stage rs1 uses M-stage rd */
        rd_m != 0 && rs1_x == rd_m
        /* Skip M-stage instructions which don't have rd */
        && opcode_m != `OP_BRANCH && opcode_m != `OP_STORE && opcode_m != `OP_ECALL && opcode_m != `OP_NOOP && opcode_m != `OP_FENCE;
    assign data_rs2_mx_bypass =
        /* X-stage rs2 uses M-stage rd */
        rd_m != 0 && rs2_x == rd_m
        /* Skip M-stage instructions which don't have rd */
        && opcode_m != `OP_BRANCH && opcode_m != `OP_STORE && opcode_m != `OP_ECALL && opcode_m != `OP_NOOP && opcode_m != `OP_FENCE;
    assign data_rs1_wx_bypass =
        /* X-stage rs1 uses W-stage rd */
        rd_w != 0 && rs1_x == rd_w
        /* Skip W-stage instructions which don't have rd */
        && opcode_w != `OP_BRANCH && opcode_w != `OP_STORE && opcode_w != `OP_ECALL && opcode_w != `OP_NOOP && opcode_w != `OP_FENCE;
    assign data_rs2_wx_bypass =
        /* X-stage rs1 uses W-stage rd */
        rd_w != 0 && rs2_x == rd_w
        /* Skip W-stage instructions which don't have rd */
        && opcode_w != `OP_BRANCH && opcode_w != `OP_STORE && opcode_w != `OP_ECALL && opcode_w != `OP_NOOP && opcode_w != `OP_FENCE;
    assign data_rs2_wm_bypass =
        /* M-stage store's data uses W-stage rd */
        opcode_m == `OP_STORE && rd_w != 0 && rs2_m == rd_w
        /* Skip W-stage instructions which don't have rd */
        && opcode_w != `OP_BRANCH && opcode_w != `OP_STORE && opcode_w != `OP_ECALL && opcode_w != `OP_NOOP && opcode_w != `OP_FENCE;

    /* Detect kill conditions */
    wire branch_or_jalr_taken;
    wire jal_taken;

    assign branch_or_jalr_taken =
        /* Detected in X-stage, kills current F and D instructions */
        opcode_x == `OP_BRANCH && PCSel == `PCSEL_ALU || opcode_x == `OP_JALR;
    assign jal_taken =
        /* Detected in D-stage, kills current F instruction */
        opcode == `OP_JUMP;

    /* Detect stall conditions */
    wire load_use_stall;
    wire decode_writeback_stall;

    assign decode_writeback_stall =
        /* Skip D-stage instructions which don't have rs1 or rs2 */
        opcode != `OP_NOOP && opcode != `OP_LUI && opcode != `OP_AUIPC && opcode != `OP_JUMP && opcode != `OP_ECALL
        /* Skip W-stage instructions which don't have rd */
        && opcode_w != `OP_BRANCH && opcode_w != `OP_STORE && opcode_w != `OP_ECALL && opcode_w != `OP_NOOP && opcode_w != `OP_FENCE
        && rd_w != 0
        && (rs1 == rd_w || rs2 == rd_w && opcode != `OP_LOAD && opcode != `OP_IMM)
        /* Branches, jalr and jal resolved in the X-stage override decode_writeback_stall */
        && !branch_or_jalr_taken && !jal_taken;
    assign load_use_stall =
        opcode != `OP_NOOP && opcode != `OP_LUI && opcode != `OP_AUIPC && opcode != `OP_JUMP && opcode != `OP_ECALL
        && opcode_x == `OP_LOAD && rd_x != 0 && (rs1 == rd_x || rs2 == rd_x && opcode != `OP_STORE && opcode != `OP_LOAD && opcode != `OP_IMM);

    /* Branch compare and control units */
    wire PCSel;
    wire BrUn, BrEq, BrLt;
    wire ASel, BSel;
    wire [3:0] ALUSel;
    wire MemRW; reg MemRW_m;
    wire [1:0] WBSel; reg [1:0] WBSel_m;
    wire [1:0] dmem_width; reg [1:0] dmem_width_m;
    wire dmem_signed_read; reg dmem_signed_read_m;
    wire RegWEn;
    branch_compare brcp
    (
        .data_rs1(data_rs1_mx_bypass ? alu_out_m : (data_rs1_wx_bypass ? data_rd_w : data_rs1_x)), /* in */
        .data_rs2(data_rs2_mx_bypass ? alu_out_m : (data_rs2_wx_bypass ? data_rd_w : data_rs2_x)), /* in */
        .BrUn(BrUn), /* in */
        .BrEq(BrEq),
        .BrLt(BrLt)
    );
    control ctrl
    (
        .opcode(opcode_x), /* in from X-stage */
        .funct3(funct3_x), /* in from X-stage */
        .funct7(funct7_x), /* in from X-stage */
        .PCSel(PCSel),
        .RegWEn(RegWEn),
        .BrUn(BrUn),
        .BrEq(BrEq), /* in */
        .BrLt(BrLt), /* in */
        .ASel(ASel),
        .BSel(BSel),
        .ALUSel(ALUSel),
        .MemRW(MemRW),
        .WBSel(WBSel),
        .mem_width(dmem_width),
        .mem_signed_read(dmem_signed_read)
    );

    /* Arithmetic and logic unit */
    wire [31:0] alu_a, alu_b, alu_out;
    alu arth
    (
        .ALUSel(ALUSel),
        .a(alu_a),
        .b(alu_b),
        .out(alu_out)
    );
    always @(posedge clock) begin
        alu_out_m <= alu_out;
    end

    /* ASel and BSel muxes */
    assign alu_a = ASel == `ASEL_REG ? (data_rs1_mx_bypass ? alu_out_m : (data_rs1_wx_bypass ? data_rd_w : data_rs1_x)) : pc_x;
    assign alu_b = BSel == `BSEL_REG ? (data_rs2_mx_bypass ? alu_out_m : (data_rs2_wx_bypass ? data_rd_w : data_rs2_x)) : imm_x;

    /* Pipeline various control signals */
    always @(posedge clock) begin
        RegWEn_m <= RegWEn; RegWEn_w <= RegWEn_m;
        WBSel_m <= WBSel;
        MemRW_m <= MemRW;
        dmem_width_m <= dmem_width;
        dmem_signed_read_m <= dmem_signed_read;
    end

    /* Set inst_d based on taken load use, conditional branches and unconditional jumps */
    always @(posedge clock) begin
        if (!load_use_stall && !decode_writeback_stall) begin
            if (branch_or_jalr_taken) // note that conditional branch taken overrides unconditional jump
                inst_d <= 32'h0000_0000; // kill current F-stage instruction
            else if (jal_taken)
                inst_d <= 32'h0000_0000; // kill current F-stage instruction
            else
                inst_d <= imem_data_out; // normal
        end /* Stall F and D one cycle when load use or decode writeback dependency detected */
    end

    /* Set opcode, opcode_x and opcode_m based on load use and taken conditional branches */
    always @(posedge clock) begin
        if (load_use_stall || decode_writeback_stall)
            opcode_x <= `OP_NOOP; // insert bubble
        else if (branch_or_jalr_taken)
            opcode_x <= `OP_NOOP; // kill current D-stage instruction
        else
            opcode_x <= opcode;
        opcode_m <= opcode_x; opcode_w <= opcode_m;
    end

    /* ---------------
       4. MEMORY STAGE
       --------------- */
    /* Data memory */
    wire [31:0] dmem_data_out;
    memory dmem
    (
        .clock(clock),
        .address(alu_out_m),
        .data_in(data_rs2_wm_bypass ? data_rd_w : data_rs2_m),
        .data_out(dmem_data_out),
        .read_write(MemRW_m),
        .width(dmem_width_m),
        .signed_read(dmem_signed_read_m)
    );

    /* ------------------
       5. WRITEBACK STAGE
       ------------------ */
    /* WBSel mux */
    always @(posedge clock) begin
        case (WBSel_m)
            `WBSEL_PC4: data_rd_w <= pc_m + 4;
            `WBSEL_ALU: data_rd_w <= alu_out_m;
            `WBSEL_MEM: data_rd_w <= dmem_data_out;
            default: data_rd_w = 32'bX;
        endcase
    end

    assign pc_f_d_stall = !load_use_stall && !decode_writeback_stall;

    /* PCSel mux */
    wire [31:0] next_pc;
    always @(posedge clock) begin
        if (pc_f_d_stall) begin
            if (PCSel == `PCSEL_ALU && opcode_x != `OP_JUMP)
                pc_f <= alu_out;
            else if (opcode == `OP_JUMP)
                pc_f <= pc_d + imm;
            else
                pc_f <= pc_f + 4;
        end
    end

    initial begin
        $dumpfile("simple_programs_testbench.vcd");
        $dumpvars(0, regf);
    end

    integer i;
    always @(posedge clock) begin
        $display(">>> pc_f=0x%h", pc_f);
        $display("    pc_d=0x%h rd=%d rs1=%d data_rs1=0x%h rs2=%d data_rs2=0x%h", pc_d, rd, rs1, data_rs1, rs2, data_rs2);
        $display("    pc_x=0x%h", pc_x);
        case (opcode_x)
            `OP_REG: $display("        [R-type] rd_x=%d rs1_x=%d rs2_x=%d alu_a=0x%h alu_b=0x%h alu_out=0x%h",
                rd_x, rs1_x, rs2_x, alu_a, alu_b, alu_out);
            `OP_IMM: $display("        [I-type] rd_x=%d rs1_x=%d alu_a=0x%h imm_x=%d alu_out=0x%h",
                rd_x, rs1_x, alu_a, $signed(imm_x), alu_out);
            `OP_LOAD: $display("        [load]");
            `OP_JALR: $display("        [jalr] alu_a=0x%h imm=0x%h alu_out=0x%h",
                alu_a, $signed(imm_x), alu_out);
            `OP_STORE: $display("        [S-type]");
            `OP_BRANCH: $display("        [B-type] BrUn=%d BrEq=%d BrLt=%d effective_address=0x%h brcp.data_rs1=0x%h brcp.data_rs2=0x%h",
                BrUn, BrEq, BrLt, alu_out, brcp.data_rs1, brcp.data_rs2);
            `OP_LUI: $display("        [lui] rd_x=%d imm=0x%h",
                rd_x, $signed(imm_x));
            `OP_AUIPC: $display("        [auipc] rd_x=%d pc_x=0x%h imm_x=0x%h",
                rd_x, pc_x, imm_x);
            `OP_JUMP: $display("        [jal] rd_x=%d next_pc(alu_out)=0x%h",
                rd_x, alu_out);
            `OP_FENCE: $display("        [fence]");
            `OP_NOOP: $display("        [noop]");
            `OP_ECALL: $display("        [ecall]");
            default: $display("        [unknown]");
        endcase
        $display("    pc_m=0x%h rd_m=%d dmem_addr(alu_out_m)=0x%h MemRW_m=%d, dmem.data_in=0x%h dmem.data_out=0x%h WBSel_m=%d", pc_m, rd_m, alu_out_m, MemRW_m, dmem.data_in, dmem.data_out, WBSel_m);
        $display("    pc_w=0x%h rd_w=%d data_rd_w=0x%h RegWEn_w=%d", pc_w, rd_w, data_rd_w, RegWEn_w);
        $display("    data_rs1_mx_bypass=%d data_rs1_wx_bypass=%d data_rs2_mx_bypass=%d data_rs2_wx_bypass=%d data_rs2_wm_bypass=%d",
            data_rs1_mx_bypass, data_rs1_wx_bypass, data_rs2_mx_bypass, data_rs2_wx_bypass, data_rs2_wm_bypass);
        $display("    branch_or_jalr_taken=%d jal_taken=%d", branch_or_jalr_taken, jal_taken);
        $display("    pc_f_d_stall=%d decode_writeback_stall=%d load_use_stall=%d", pc_f_d_stall, decode_writeback_stall, load_use_stall);
        $display("    PCSel=%d", PCSel);

        if (opcode_w == `OP_JALR && pc_w == 32'h010f4240) begin
            /* Dump final register contents */
            $display("[REGISTERS] Final register contents");
            for (i=0; i<32; i=i+1)
                $display("[REGISTERS] x%d=%d", i, regf.array[i]);
            $finish;
        end
    end
endmodule
