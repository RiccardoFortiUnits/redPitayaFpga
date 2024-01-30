`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.01.2024 16:55:25
// Design Name: 
// Module Name: tensionShifter
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

//shift range [-1V,1V] to [-1V,0V]

module tensionShifter#(
    parameter totalBits = 14
)(
    input signed [totalBits-1:0] in,
    output wire signed [totalBits-1:0] out
);
    assign out = (in >> 1) - (1 << (totalBits-2));
    
endmodule
