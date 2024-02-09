`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.02.2024 10:17:30
// Design Name: 
// Module Name: blinker
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


module blinker#(
    parameter blinkClockCycles = 12500000
)(
    input clk,
    input enable,
    output reg out
);
    
    localparam nOfBits = $clog2(blinkClockCycles);
    
    reg [nOfBits:0] counter;
    
    always @(posedge clk)begin
        if(enable) begin
            out <= 0;
            counter <= 0;
        end else begin
            if($unsigned(counter) >= $unsigned(blinkClockCycles))begin
                counter <= 0;
                out <= ~out;
            end else begin
                counter <= counter + 1;
            end
        end
    end
    
endmodule
