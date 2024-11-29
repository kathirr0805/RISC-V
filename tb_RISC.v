module RISC_Testbench;
    // Inputs
    reg clk;
    reg reset;

    // Instantiate the RISC processor
    risc uut (
        .clk(clk), 
        .reset(reset)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end

    // Initialize inputs and check outputs
    initial begin
        // Initialize Reset
        reset = 1;
        #10;
        reset = 0;

        // Observe outputs
        $monitor("Time = %0t, PC = %h, Instruction = %h, ALU_Result = %h, Rd1 = %h, Rd2 = %h", 
                 $time, uut.PC_top, uut.instruction_top, uut.ALU_Result_top, uut.Rd1_top, uut.Rd2_top);

        // Let the simulation run for a while
        #1000;

        // Stop the simulation
        $stop;
    end
endmodule
