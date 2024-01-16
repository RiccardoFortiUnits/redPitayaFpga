`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09.01.2024 14:41:36
// Design Name: 
// Module Name: downsampler
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


module downsampler#(
    parameter downsampling = 10
)(
    input [13:0] in,
    output [13:0] out,
    input clk_in,
    output reg clk_out
);
    ir_filter_fixed #(
        0.5/downsampling,
        30,
        30-14
    )irff(
        clk_in,
        in,
        out
    );
    
    reg [15:0] currentClockPosition;
    
    always @(posedge(clk_in))begin
        if(currentClockPosition == 0)begin
            clk_out <= 1;
        end
    end
    always @(negedge(clk_in))begin
        if(currentClockPosition == 0)begin
            clk_out <= 0;
            currentClockPosition <= currentClockPosition + 1;
        end
        if(currentClockPosition == downsampling)begin
            currentClockPosition <= 0;
        end
    end
    
    
    
endmodule
