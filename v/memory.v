`define BASE_ADDRESS 32'h0100_0000 /* start of text segment */
`define DEFAULT_MEM_DEPTH 10**6 /* default number of bytes in the memory array */
`define INITIAL_DATA_DIR "simple-programs"
`define INITIAL_DATA_FILE "demo-monday-august-3.x"

`define MEM_WIDTH_B 2'b01
`define MEM_WIDTH_H 2'b10
`define MEM_WIDTH_W 2'b11

/* Byte-addessable memory module which allows (signed or unsigned) 8/16/32-bit
   combinational reads and synchronous writes. */
module memory
#
(
    parameter DEPTH = `DEFAULT_MEM_DEPTH
)
(
    input wire clock,
    input wire [31:0] address,
    input wire [31:0] data_in,
    output wire [31:0] data_out,
    input wire read_write,
    input wire [1:0] width,
    input wire signed_read
);
    reg [7:0] array [DEPTH-1:0];

    /* Sequential write when enabled.
       Note that out of bounds array writes do nothing during simulation. */
    always @(posedge clock) begin
        if (read_write == 1'b0) begin
            case (width)
                `MEM_WIDTH_B:
                    begin
                        array[address-`BASE_ADDRESS] <= data_in[7:0];
                    end
                `MEM_WIDTH_H:
                    begin
                        array[address-`BASE_ADDRESS+1] <= data_in[15:8];
                        array[address-`BASE_ADDRESS] <= data_in[7:0];
                    end
                default:
                    begin
                        array[address-`BASE_ADDRESS+3] <= data_in[31:24];
                        array[address-`BASE_ADDRESS+2] <= data_in[23:16];
                        array[address-`BASE_ADDRESS+1] <= data_in[15:8];
                        array[address-`BASE_ADDRESS] <= data_in[7:0];
                    end
            endcase
        end
    end

    /* Combinational read when enabled.
       Note that out of bounds array reads lead to X during simulation.

       TODO: iverilog hangs when using a case statement, so an assign statement
             is needed.  */
    assign data_out = read_write == 1'b0 ? 32'bX :
        width == `MEM_WIDTH_B ?
            (signed_read ? {{24{array[address-`BASE_ADDRESS][7]}}, array[address-`BASE_ADDRESS]} : array[address-`BASE_ADDRESS]) :
            width == `MEM_WIDTH_H ?
                (signed_read ? {{16{array[address-`BASE_ADDRESS+1][7]}}, array[address-`BASE_ADDRESS+1], array[address-`BASE_ADDRESS]} :
                {array[address-`BASE_ADDRESS+1], array[address-`BASE_ADDRESS]}) :
                    {
                        array[address-`BASE_ADDRESS+3],
                        array[address-`BASE_ADDRESS+2],
                        array[address-`BASE_ADDRESS+1],
                        array[address-`BASE_ADDRESS]
                    };

    /* Read initial data from file into temporary buffer (32 bit words) and
       then copy into array (8 bit words). The buffer is actually larger than
       the array due to its larger word size, but it's assumed that the array
       is large enough to hold the contents of each file.

       I don't bother checking the file's size (although a testbench will),
       rather we just copy the whole contents of buffer into array. Out of
       bounds writes during Verilog simulation are no-ops. */
    reg [31:0] buffer [DEPTH-1:0];
    integer i;

    initial begin
        $readmemh({`INITIAL_DATA_DIR, "/", `INITIAL_DATA_FILE}, buffer);
        $display("[MEMORY] Sanity check 1: %h", {buffer[0]});
        for (i=0; i<DEPTH; i=i+1) begin
            array[i*4+3] = buffer[i][31:24];
            array[i*4+2] = buffer[i][23:16];
            array[i*4+1] = buffer[i][15:8];
            array[i*4] = buffer[i][7:0];
        end
        $display("[MEMORY] Sanity check 2: %h", {array[3], array[2], array[1], array[0]});
        if ({buffer[0]} != {array[3], array[2], array[1], array[0]}) begin
            $display("%h", {buffer[0]});
            $display("%h", {array[3], array[2], array[1], array[0]});
            $display("[MEMORY] Sanity check failed");
            $finish;
        end
    end
endmodule
