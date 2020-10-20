/* An array of 32 32-bit registers with combinational read and synchronous
   write. */
module register_file
(
    input wire clock,
    input wire [4:0] addr_rs1,
    input wire [4:0] addr_rs2,
    input wire [4:0] addr_rd,
    input wire [31:0] data_rd,
    output wire [31:0] data_rs1,
    output wire [31:0] data_rs2,
    input wire write_enable
);
    reg [31:0] array [31:0];

    assign data_rs1 = array[addr_rs1];
    assign data_rs2 = array[addr_rs2];

    always @(posedge clock) begin
        if (write_enable && addr_rd != 0) begin
            array[addr_rd] <= data_rd;
        end
    end

    /* In PD4, the stack pointer should be initialized to the end of memory. */
    integer i;
    initial begin
        $display("[REGISTERS] Initial register contents");
        for (i=0; i<32; i++) begin
            if (i == 2) begin
                array[i] = 32'h0100_0000 + 10**6;
            end else begin
                array[i] = i;
            end
            $display("[REGISTERS] x%d=%d", i, array[i]);
        end
    end
endmodule
