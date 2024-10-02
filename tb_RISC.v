`timescale 1ns/1ps

module testbench;
    reg clk;
    reg reset;
    wire [31:0] PC_out;

    // Instantiate the top module
    top uut (
        .clk(clk),
        .reset(reset)
    );

    initial begin
        // Dump file setup
        $dumpfile("RISC.vcd");  // Specify the VCD file name
        $dumpvars(0, uut);      // Dump all variables in the top module

        // Initialize
        clk = 0;
        reset = 1;

        // Release reset
        #10;
        reset = 0;

        // Run the simulation for a certain period
        #1000;

        // Stop simulation
        $finish;
    end

    // Clock generation
    always #5 clk = ~clk;  // 100MHz clock

    // Monitor signals
    initial begin
        $monitor("Time: %t | PC_out: %h", $time, uut.PC_top);
    end
endmodule
