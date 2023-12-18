`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.12.2023 12:19:28
// Design Name: 
// Module Name: movingAverage
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


module ir_filter(
    input clk_i,
    input [13:0] in,
    output [13:0] out
    );
    parameter decimalBits = 8;
      // Function to convert floating-point to fixed-point
  function signed [13+decimalBits:0] convertToFixedPoint(real value);
    convertToFixedPoint = $rtoi(value * (1 << decimalBits));
  endfunction
    reg signed [13+decimalBits:0] y_delayed = 0; // Delayed output for feedback

  // Coefficients with decimal bits
  reg signed [13+decimalBits:0] a = convertToFixedPoint(0.8); // Example 'a' with 4 decimal bits
  reg signed [13+decimalBits:0] b = convertToFixedPoint(0.2); // Example 'b' with 4 decimal bits

  always @(posedge clk_i) begin
      y_delayed <= ((a * y_delayed) + (b * in)) >> decimalBits;
  end
  
  //assign out = y_delayed[13+decimalBits:decimalBits];
  assign out = in;
    
endmodule
