module Instruction_Mem(
    input clk,
    input reset,
    input [31:0] read_address,  // 32-bit address input
    output reg [31:0] instruction_out, // 32-bit instruction output
    input [31:0] write_address, // Address for loading instructions
    input [31:0] write_data,    // Data to write into memory
    input write_enable          // Control signal for writing instructions
);

    reg [31:0] I_Mem[63:0]; // 64 memory locations of 32-bit width
    integer k;

    // Initialize memory with sample instructions and zeros
    initial begin
        I_Mem[0] = 32'h00000093; // NOP: ADD x1, x0, x0
        I_Mem[1] = 32'h00100113; // ADDI x2, x0, 1
        I_Mem[2] = 32'h00208193; // ADDI x3, x1, 2
        I_Mem[3] = 32'h00310213; // ADDI x4, x2, 3
        // Initialize the rest of the memory to zero
        for (k = 4; k < 64; k = k + 1) begin
            I_Mem[k] = 32'b0;
        end
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset memory to zero
            for (k = 0; k < 64; k = k + 1) begin
                I_Mem[k] <= 32'b0;
            end
        end else if (write_enable) begin
            // Write data into memory at specified address
            I_Mem[write_address[5:0]] <= write_data; 
            // Using only the lower 6 bits to index memory
        end else begin
            // Read memory at indexed address
            instruction_out <= I_Mem[read_address[5:0]];
        end
    end
endmodule
