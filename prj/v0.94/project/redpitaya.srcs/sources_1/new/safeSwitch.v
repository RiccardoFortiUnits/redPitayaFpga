`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.02.2024 10:17:30
// Design Name: 
// Module Name: safeSwitch
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


module safeSwitch(
    input saturation,
    input clk,
    input reset,
    input [1:0] configuration,
    input [7:0] saturationTime,
    output reg outReset
);
    
    parameter disabled          = 0,
              stopAtSaturation  = 1,
              resetAtSaturation = 2;
              
    wire currentSaturation;
    reg [7:0] currentSaturationTime;
    
    assign currentSaturation = $unsigned(currentSaturationTime) >= $unsigned(saturationTime);
                
    always @(posedge clk)begin
        if(reset)begin
            outReset <= 0;
            currentSaturationTime <= 0;
        end else begin
            if (saturation)begin
                currentSaturationTime <= currentSaturation ? currentSaturationTime : currentSaturationTime+1;
            end else begin
                currentSaturationTime <= 0;
            end
            casez (configuration)
                disabled          : begin outReset <= 0;                            end
                stopAtSaturation  : begin outReset <= outReset | currentSaturation; end
                resetAtSaturation : begin outReset <= currentSaturation;            end
            endcase
        end
    end
    
endmodule
