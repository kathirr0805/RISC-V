// Multiplexers
module Mux1(sel1, A1, B1, Mux1_out);
    input sel1;
    input [31:0] A1, B1;
    output [31:0] Mux1_out;
    assign Mux1_out = (sel1 == 1'b0) ? A1 : B1;
endmodule 