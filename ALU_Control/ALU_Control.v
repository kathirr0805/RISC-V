// ALU Control 
module ALU_Control (ALUOp, fun7, fun3, Control_out);
    input fun7;
    input [2:0] fun3;
    input [1:0] ALUOp;
    output reg [3:0] Control_out;

    always @(*) begin
        case({ALUOp, fun7, fun3})
            6'b00_0_000: Control_out = 4'b0010;  // ADD
            6'b01_0_000: Control_out = 4'b0110;  // SUB
            6'b10_0_000: Control_out = 4'b0010;  // ADD
            6'b10_1_000: Control_out = 4'b0110;  // SUB
            6'b10_0_111: Control_out = 4'b0000;  // AND
            6'b10_0_110: Control_out = 4'b0001;  // OR
            default: Control_out = 4'b0000; // Default
        endcase
    end
endmodule