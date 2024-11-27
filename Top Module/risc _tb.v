module risc_tb;
    reg clk, reset;
    wire [31:0] PC_top;

    // Instantiate the risc processor
    risc uut (
        .clk(clk),
        .reset(reset)
    );

    // Clock Generation
    always begin
        #5 clk = ~clk;
    end

    // Test Procedure
    initial begin
        // Initialize Signals
        clk = 0;
        reset = 1;
        #10 reset = 0;
        #100 $finish;
    end
endmodule
