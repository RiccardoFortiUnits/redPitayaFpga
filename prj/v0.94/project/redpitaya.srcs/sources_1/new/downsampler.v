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
    parameter     totalBits = 14,
    parameter     fracBits = 0,
    parameter downsampling = 10
)(
    input [totalBits-1:0] in,
    output reg [totalBits-1:0] out,
    input clk_in,
    output reg clk_out,
    input reset
);

    wire [totalBits-1:0] outWire;

    ir_filter_fixed #(
        0.5/downsampling,
        totalBits,
        fracBits,
        totalBits+8,//todo is it enough? (+3 for whole part, +5 for fractional part) 
        fracBits+5
    )irff(
        clk_in,
        reset,
        in,
        outWire
    );
    
    reg [$clog2(downsampling):0] currentClockPosition;
    
    always @(posedge clk_in)begin
        if(reset) begin
            clk_out <= 0;
            currentClockPosition <= 0;
            out <= 0;
        end else begin            
            if(currentClockPosition == downsampling - 1)begin
                currentClockPosition <= 0;
                clk_out <= 1;
                out <= outWire;
            end else begin
                currentClockPosition <= currentClockPosition + 1;
                clk_out <= 0;//currentClockPosition == 0;
            end
        end
    end
    
    always @(negedge clk_in)begin
        clk_out <= 0;
    end
    
    
    
endmodule


module clockDivider#(
    parameter downsampling = 10
)(
    input clk_in,
    output reg clk_out,
    input reset
);
    
    reg [$clog2(downsampling):0] currentClockPosition;
    
    always @(posedge clk_in)begin
        if(reset) begin
            clk_out <= 0;
            currentClockPosition <= 0;
        end else begin            
            if(currentClockPosition == downsampling - 1)begin
                currentClockPosition <= 0;
                clk_out <= 1;
            end else begin
                currentClockPosition <= currentClockPosition + 1;
                clk_out <= 0;//currentClockPosition == 0;
            end
        end
    end
    
    always @(negedge clk_in)begin
        clk_out <= 0;
    end
    
    
    
endmodule
