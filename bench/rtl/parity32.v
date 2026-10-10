module parity32(input [31:0] x, output p);
  assign p = ^x;
endmodule
