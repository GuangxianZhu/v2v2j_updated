module popcnt16(input [15:0] x, output [4:0] n);
  integer i;
  reg [4:0] c;
  always @* begin c = 0; for (i = 0; i < 16; i = i + 1) c = c + x[i]; end
  assign n = c;
endmodule
