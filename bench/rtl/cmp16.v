module cmp16(input [15:0] a, input [15:0] b, output lt, output eq);
  assign lt = a < b;
  assign eq = a == b;
endmodule
