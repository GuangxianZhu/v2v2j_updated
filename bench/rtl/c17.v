// ISCAS-85 c17（6 个 NAND）
module c17(input N1, input N2, input N3, input N6, input N7, output N22, output N23);
  wire N10 = ~(N1 & N3), N11 = ~(N3 & N6), N16 = ~(N2 & N11), N19 = ~(N11 & N7);
  assign N22 = ~(N10 & N16);
  assign N23 = ~(N16 & N19);
endmodule
