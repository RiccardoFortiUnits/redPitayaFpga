`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.03.2024 12:26:29
// Design Name: 
// Module Name: triggerForOneClock
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


module triggerForOneClock(
    input clk,
    input reset,
    input in,
    output reg out
);

reg prevVal;

always @(posedge clk)begin
    if(reset)begin
        out <= 0;
        prevVal <= 0;
    end else begin
        prevVal <= in;
        out <= in & !prevVal;
    end 
end

endmodule
