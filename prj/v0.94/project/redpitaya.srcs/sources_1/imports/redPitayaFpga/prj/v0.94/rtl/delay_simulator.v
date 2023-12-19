`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.12.2023 16:02:31
// Design Name: 
// Module Name: delay_simulator
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


module delay_simulator#(
    parameter nOfCycles = 300
)(
    input clk,
    input [14:0] in,
    output [14:0] out
    );
    reg [15:0] delay_buffer [0:nOfCycles]; // Adjust the range based on your maximum delay

    integer i;
  
  always @(posedge clk) begin
  
      // Shift data through the delay buffer
      for (i = nOfCycles - 1; i > 0; i = i - 1) begin
        delay_buffer[i] <= delay_buffer[i - 1];
      end
    
      // Store the current input in the delay buffer
      delay_buffer[0] <= in;
    
      // Output the delayed data after N cycles
  end
  
assign out = delay_buffer[nOfCycles - 1];
    
    
endmodule
