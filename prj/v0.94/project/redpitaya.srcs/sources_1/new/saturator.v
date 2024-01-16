`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.01.2024 15:49:02
// Design Name: 
// Module Name: saturator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module saturator #(parameter s = 8, parameter n = 3) (
  input wire signed [s-1:0] input_data,
  output reg signed [s-1:0] saturated_output
);

  // Calculate the saturation limit
  always @(*) begin
    if ({input_data [s-1],(|input_data [s-2:n-1])} == 'b01) begin// positive saturation
        saturated_output <= (1<<(n-1)) - 1; // max positive
    end else if ({input_data [s-1],(&input_data [s-2:n-1])} == 'b10) begin // negative saturation
        saturated_output <= -$signed(1<<(n-1)); // max negative
    end else begin// No saturation
        saturated_output = input_data;
    end
  end
endmodule
