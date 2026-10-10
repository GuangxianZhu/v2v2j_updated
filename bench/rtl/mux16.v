module mux16(input [15:0] d, input [3:0] s, output y);
  assign y = d[s];
endmodule
