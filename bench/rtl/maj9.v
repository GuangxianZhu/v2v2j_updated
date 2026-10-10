module maj9(input [8:0] x, output y);
  integer i; reg [3:0] c;
  always @* begin c = 0; for (i = 0; i < 9; i = i + 1) c = c + x[i]; end
  assign y = c >= 5;
endmodule
