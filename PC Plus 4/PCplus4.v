module PCplus4(fromPC, NextoPC);
    input [31:0] fromPC;
    output [31:0] NextoPC;
    assign NextoPC = fromPC + 4;
endmodule 