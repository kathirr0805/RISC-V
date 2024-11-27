module risc(clk, reset);
    input clk, reset;
    wire [31:0] PC_top, instruction_top;
    wire RegWrite_top, ALUSrc_top, zero_top, MemRead_top, MemtoReg_top, MemWrite_top, branch_top;
    wire [1:0] ALUOp_top;
    wire [3:0] control_top;
    wire [31:0] Rd1_top, Rd2_top, ImmExt_top, ALU_Result_top, Mux1_out, Mux3_out, MemData_top, NextoPC;

    // Program Counter
    Program_Counter PC (.clk(clk), .reset(reset), .PC_in(NextoPC), .PC_out(PC_top));

    // PC Adder
    PCplus4 PC_Adder (.fromPC(PC_top), .NextoPC(NextoPC));

    // Instruction Memory
    Instruction_Mem Inst_Memory (.clk(clk), .reset(reset), .read_address(PC_top), .instruction_out(instruction_top));

    // Register File
    Reg_File Reg_File (.clk(clk), .reset(reset), .RegWrite(RegWrite_top), .Rs1(instruction_top[19:15]), .Rs2(instruction_top[24:20]), .Rd(instruction_top[11:7]), .Write_data(Mux3_out), .read_data1(Rd1_top), .read_data2(Rd2_top));

    // Immediate Generator
    ImmGen ImmGen (.Opcode(instruction_top[6:0]), .instruction(instruction_top), .ImmExt(ImmExt_top));

    // Control Unit
    Control_Unit Control_Unit (.instruction(instruction_top[6:0]), .Branch(branch_top), .MemRead(MemRead_top), .MemtoReg(MemtoReg_top), .ALUOp(ALUOp_top), .MemWrite(MemWrite_top), .ALUSrc(ALUSrc_top), .RegWrite(RegWrite_top));

    // ALU Control
    ALU_Control ALU_Control (.ALUOp(ALUOp_top), .fun7(instruction_top[30]), .fun3(instruction_top[14:12]), .Control_out(control_top));

    // ALU
    ALU_unit ALU (.A(Rd1_top), .B(Mux1_out), .Control_in(control_top), .ALU_Result(ALU_Result_top), .zero(zero_top));

    // ALU Mux
    Mux1 ALU_mux (.sel1(ALUSrc_top), .A1(Rd2_top), .B1(ImmExt_top), .Mux1_out(Mux1_out));

    // Adder for Branch Address
    Adder Branch_Adder (.in_1(PC_top), .in_2(ImmExt_top), .Sum_out(NextoPC));

    // AND Gate
    wire and_out; // Declare wire for AND output
    AND_logic AND (.branch(branch_top), .zero(zero_top), .and_out(and_out));  // Connect to AND logic

    // Mux for PC
    Mux2 PC_mux (.sel2(and_out), .A2(NextoPC), .B2(PC_top), .Mux2_out(NextoPC));

    // Data Memory
    Data_Memory Data_mem (.clk(clk), .reset(reset), .MemWrite(MemWrite_top), .MemRead(MemRead_top), .read_address(ALU_Result_top), .Write_data(Rd2_top), .MemData_out(MemData_top));

    // Mux for MemToReg
    Mux3 Mem_to_Reg_mux (.sel3(MemtoReg_top), .A3(ALU_Result_top), .B3(MemData_top), .Mux3_out(Mux3_out));
endmodule
