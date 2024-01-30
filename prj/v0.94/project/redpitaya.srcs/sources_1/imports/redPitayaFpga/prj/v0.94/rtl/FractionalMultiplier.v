

module FractionalMultiplier #(parameter A_WIDTH = 16,
                              parameter B_WIDTH = 16,
                              parameter OUTPUT_WIDTH = 16,
                              parameter FRAC_BITS_A = 4,
                              parameter FRAC_BITS_B = 4,
                              parameter FRAC_BITS_OUT = 8) (
  input wire signed [A_WIDTH-1:0] a,
  input wire signed [B_WIDTH-1:0] b,
  output wire signed [OUTPUT_WIDTH-1:0] result
);
  wire signed [A_WIDTH + B_WIDTH - 1:0] full_aByb;
 
  assign full_aByb = $signed(a) * $signed(b);
  
  assign result = full_aByb[OUTPUT_WIDTH - 1 + FRAC_BITS_A + FRAC_BITS_B - FRAC_BITS_OUT: 
                                               FRAC_BITS_A + FRAC_BITS_B - FRAC_BITS_OUT];

endmodule

module clocked_FractionalMultiplier #(
                              parameter A_WIDTH = 16,
                              parameter B_WIDTH = 16,
                              parameter OUTPUT_WIDTH = 16,
                              parameter FRAC_BITS_A = 4,
                              parameter FRAC_BITS_B = 4,
                              parameter FRAC_BITS_OUT = 8) (
  input wire clk,
  input wire signed [A_WIDTH-1:0] a,
  input wire signed [B_WIDTH-1:0] b,
  output wire signed [OUTPUT_WIDTH-1:0] result
);
  reg signed [A_WIDTH + B_WIDTH - 1:0] full_aByb;
 
  always @(posedge clk)
    full_aByb <= $signed(a) * $signed(b);
  
  assign result = full_aByb[OUTPUT_WIDTH - 1 + FRAC_BITS_A + FRAC_BITS_B - FRAC_BITS_OUT: 
                                               FRAC_BITS_A + FRAC_BITS_B - FRAC_BITS_OUT];

endmodule
