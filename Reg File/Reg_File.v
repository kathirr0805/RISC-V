module Reg_File (
    input clk,
    input reset,
    input RegWrite,
    input [4:0] Rs1, Rs2, Rd,  // Register addresses
    input [31:0] Write_data,   // Data to write
    output [31:0] read_data1, read_data2 // Outputs for read ports
);

    reg [31:0] Registers[31:0]; // 32 registers, 32-bit each
    integer k;

    // Reset and write logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Initialize all registers to zero
            for (k = 0; k < 32; k = k + 1) begin
                Registers[k] <= 32'b0;
            end
        end else if (RegWrite && Rd != 5'b0) begin
            // Write to register Rd if not x0 (Register 0)
            Registers[Rd] <= Write_data;
        end
    end
    // Assign read ports (combinational logic)
    assign read_data1 = Registers[Rs1];
    assign read_data2 = Registers[Rs2];

endmodule
