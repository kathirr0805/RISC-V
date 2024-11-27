module Mux3(sel3, A3, B3, Mux3_out);
    input sel3;
    input [31:0] A3, B3;
    output [31:0] Mux3_out;
    assign Mux3_out = (sel3 == 1'b0) ? A3 : B3; 
endmodule
