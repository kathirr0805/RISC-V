module Program_Counter (clk, reset, PC_in, PC_out);
    input clk, reset;
    input [31:0] PC_in;
    output reg [31:0] PC_out;

    always @(posedge clk or posedge reset) begin
        if (reset)
            PC_out <= 32'h00000000;  // Reset value
        else
            PC_out <= PC_in;
    end
endmodule

module PCplus4(fromPC, NextoPC);
    input [31:0] fromPC;
    output [31:0] NextoPC;
    assign NextoPC = fromPC + 4;
endmodule

module Instruction_Mem (
    input clk,
    input reset,
    input [31:0] read_address,
    output reg [31:0] instruction_out,
    input [31:0] write_address,
    input [31:0] write_data,
    input write_enable
);

    reg [31:0] I_Mem [63:0];
    integer i;

    // Initialize the instruction memory
    initial begin
        I_Mem[0] = 32'h00000093; // NOP
        I_Mem[1] = 32'h00100093; // ADDI x1, x0, 1  --> Load 1 into x1
        I_Mem[2] = 32'h00200113; // ADDI x2, x0, 2  --> Load 2 into x2
        I_Mem[3] = 32'h00310133; // ADD x3, x1, x2  --> Add x1 and x2, store in x3
        I_Mem[4] = 32'h403101b3; // SUB x3, x1, x2  --> Subtract x2 from x1, store in x3
        I_Mem[5] = 32'h005101b3; // AND x3, x1, x2  --> AND x1 and x2, store in x3
        I_Mem[6] = 32'h00510133; // OR  x3, x1, x2  --> OR  x1 and x2, store in x3
        I_Mem[7] = 32'h00002103; // LW  x2, 0(x0)   --> Load word from memory location 0 into x2
        I_Mem[8] = 32'h00002223; // SW  x2, 0(x0)   --> Store word in x2 into memory location 0
        I_Mem[9] = 32'h00208163; // BEQ x1, x2, label  --> Branch if x1 == x2
        I_Mem[10] = 32'h00000000; // NOP (placeholder)

        // Initialize the rest of memory with 0's
        for (i = 11; i < 64; i = i + 1) begin
            I_Mem[i] = 32'b0;
        end
    end
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            instruction_out <= 32'b0;
        end else begin
            instruction_out <= I_Mem[read_address[31:2]]; // Aligning read_address
        end
    end
endmodule

module Reg_File (
    input clk,
    input reset,
    input RegWrite,
    input [4:0] Rs1, Rs2, Rd,
    input [31:0] Write_data,
    output [31:0] read_data1, read_data2
);

    reg [31:0] Registers [0:31];
    integer k;

    initial begin
        for (k = 0; k < 32; k = k + 1) begin
            Registers[k] = 32'b0;
        end
        Registers[1] = 32'd5; // Initialize x1 to 5
        Registers[2] = 32'd3; // Initialize x2 to 3
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (k = 0; k < 32; k = k + 1) begin
                Registers[k] <= 32'b0;
            end
            Registers[1] <= 32'd5; // Reinitialize x1 to 5
            Registers[2] <= 32'd3; // Reinitialize x2 to 3
        end else if (RegWrite && Rd != 5'b0) begin
            Registers[Rd] <= Write_data;
        end
    end

    assign read_data1 = Registers[Rs1];
    assign read_data2 = Registers[Rs2];
endmodule

module ImmGen(Opcode, instruction, ImmExt);
    input [6:0] Opcode;
    input [31:0] instruction;
    output reg [31:0] ImmExt;

    always @(*) begin
        case (Opcode)
            7'b0010011: ImmExt = {{20{instruction[31]}}, instruction[31:20]}; // ADDI
            7'b0000011: ImmExt = {{20{instruction[31]}}, instruction[31:20]}; // LW
            7'b0100011: ImmExt = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]}; // SW
            7'b1100011: ImmExt = {{19{instruction[31]}}, instruction[31], instruction[30:25], instruction[11:8], 1'b0}; // BEQ
            default: ImmExt = 32'b0;
        endcase
    end
endmodule

module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
    input [6:0] instruction;
    output reg Branch, MemRead, MemtoReg, ALUSrc, RegWrite;
    output reg [1:0] ALUOp;
    output reg MemWrite;

    always @(*) begin
        case (instruction)
            7'b0110011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b00100010; // ADD, SUB, AND, OR
            7'b0010011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b10100000; // ADDI
            7'b0000011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b11110000; // LW
            7'b0100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b10001000; // SW
            7'b1100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b00000001; // BEQ
            default:    {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b0;
        endcase
    end
endmodule


module Control_Unit(instruction, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
    input [6:0] instruction;
    output reg Branch, MemRead, MemtoReg, ALUSrc, RegWrite;
    output reg [1:0] ALUOp;
    output reg MemWrite;

    always @(*) begin
        case (instruction)
            7'b0110011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b00100010; // ADD, SUB, AND, OR
            7'b0010011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b10100000; // ADDI
            7'b0000011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b11110000; // LW
            7'b0100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b10001000; // SW
            7'b1100011: {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b00000001; // BEQ
            default:    {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp} = 8'b0;
        endcase
    end
endmodule


module ALU_Control (ALUOp, fun7, fun3, Control_out);
    input fun7;
    input [2:0] fun3;
    input [1:0] ALUOp;
    output reg [3:0] Control_out;

    always @(*) begin
        case({ALUOp, fun7, fun3})
            6'b00_0_000: Control_out = 4'b0010;  // ADDI
            6'b10_0_000: Control_out = 4'b0010;  // ADD
            6'b10_1_000: Control_out = 4'b0110;  // SUB
            6'b10_0_111: Control_out = 4'b0000;  // AND
            6'b10_0_110: Control_out = 4'b0001;  // OR
            default: Control_out = 4'b0000; // Default
        endcase
    end
endmodule

// ALU
module ALU_unit (A, B, Control_in, ALU_Result, zero);
    input [31:0] A, B;
    input [3:0] Control_in; 
    output reg zero;
    output reg [31:0] ALU_Result;
    always @(Control_in, A, B) begin
        case(Control_in)
            4'b0000: ALU_Result = A & B; // AND
            4'b0001: ALU_Result = A | B; // OR
            4'b0010: ALU_Result = A + B; // ADD, ADDI
            4'b0110: ALU_Result = A - B; // SUB
            default: ALU_Result = 32'b0;
        endcase
        zero = (ALU_Result == 32'b0) ? 1'b1 : 1'b0;
    end 
endmodule

module Data_Memory (
    input clk, 
    input reset, 
    input MemWrite, 
    input MemRead, 
    input [31:0] read_address, 
    input [31:0] Write_data, 
    output [31:0] MemData_out
);
    reg [31:0] D_Memory [63:0];
    integer k;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (k = 0; k < 64; k = k + 1) begin
                D_Memory[k] <= 32'b0;  // Initialize memory to zero
            end
        end else if (MemWrite) begin
            D_Memory[read_address[31:2]] <= Write_data; // Aligning read_address
        end
    end

    assign MemData_out = (MemRead) ? D_Memory[read_address[31:2]] : 32'b0; // Aligning read_address
endmodule

module risc (
    input clk, 
    input reset
);
    wire [31:0] PC_top, instruction_top;
    wire RegWrite_top, ALUSrc_top, zero_top, MemRead_top, MemtoReg_top, MemWrite_top, branch_top;
    wire [1:0] ALUOp_top;
    wire [3:0] control_top;
    wire [31:0] Rd1_top, Rd2_top, ImmExt_top, ALU_Result_top, Mux1_out, Mux3_out, MemData_top, NextoPC;

    // Program Counter
    Program_Counter PC (
        .clk(clk), 
        .reset(reset), 
        .PC_in(NextoPC), 
        .PC_out(PC_top)
    );

    // PC Adder
    PCplus4 PC_Adder (
        .fromPC(PC_top), 
        .NextoPC(NextoPC)
    );

    // Instruction Memory
    Instruction_Mem Inst_Memory (
        .clk(clk), 
        .reset(reset), 
        .read_address(PC_top), 
        .instruction_out(instruction_top)
    );

    // Register File
    Reg_File Reg_File (
        .clk(clk), 
        .reset(reset), 
        .RegWrite(RegWrite_top), 
        .Rs1(instruction_top[19:15]), 
        .Rs2(instruction_top[24:20]), 
        .Rd(instruction_top[11:7]), 
        .Write_data(Mux3_out), 
        .read_data1(Rd1_top), 
        .read_data2(Rd2_top)
    );

    // Immediate Generator
    ImmGen ImmGen (
        .Opcode(instruction_top[6:0]), 
        .instruction(instruction_top), 
        .ImmExt(ImmExt_top)
    );

    // Control Unit
    Control_Unit Control (
        .instruction(instruction_top[6:0]), 
        .Branch(branch_top), 
        .MemRead(MemRead_top), 
        .MemtoReg(MemtoReg_top), 
        .ALUOp(ALUOp_top), 
        .MemWrite(MemWrite_top), 
        .ALUSrc(ALUSrc_top), 
        .RegWrite(RegWrite_top)
    );

    // ALU Control Unit
    ALU_Control ALU_Control (
        .ALUOp(ALUOp_top), 
        .fun7(instruction_top[31:25]), 
        .fun3(instruction_top[14:12]), 
        .Control_out(control_top)
    );

    // MUX for ALU Source (ALUSrc)
    assign Mux1_out = ALUSrc_top ? ImmExt_top : Rd2_top;

    // ALU Unit
    ALU_unit ALU (
        .A(Rd1_top), 
        .B(Mux1_out), 
        .Control_in(control_top), 
        .ALU_Result(ALU_Result_top), 
        .zero(zero_top)
    );

    // Data Memory
    Data_Memory DataMemory (
        .clk(clk),
        .reset(reset),
        .MemWrite(MemWrite_top),
        .MemRead(MemRead_top),
        .read_address(ALU_Result_top),
        .Write_data(Rd2_top),
        .MemData_out(MemData_top)
    );

    // MUX for Write Back (MemtoReg)
    assign Mux3_out = MemtoReg_top ? MemData_top : ALU_Result_top;

    // PC Update Logic for Branching
    assign NextoPC = (branch_top && zero_top) ? PC_top + ImmExt_top : PC_top + 4;

endmodule




