module alu4(input [3:0] a, input [3:0] b, input [1:0] op, output reg [3:0] y, output z);
  always @* case (op)
    2'd0: y = a + b;
    2'd1: y = a - b;
    2'd2: y = a & b;
    default: y = a ^ b;
  endcase
  assign z = (y == 4'd0);
endmodule
