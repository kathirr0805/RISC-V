module Program_Counter(
    input clk,            // Clock signal
    input reset,          // Reset signal
    input [31:0] PC_in,   // Input value for the program counter
    output reg [31:0] PC_out // Output value of the program counter
);

    // On every rising edge of the clock or when reset is asserted
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC_out <= 32'b0; // Reset the program counter to 0
        end else begin
            PC_out <= PC_in; // Update the program counter with the input value
        end
    end

endmodule
