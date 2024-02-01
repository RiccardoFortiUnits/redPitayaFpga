`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.01.2024 14:28:24
// Design Name: 
// Module Name: resetEnlarger
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


module resetEnlarger#(
    parameter nOfDelays = 2
)(
    input reset,
    input clk,
    output outReset
);

    reg unsigned [$clog2(nOfDelays)+1:0] counter;
    reg enlargedReset;
    always @(posedge clk,reset)begin
        if(reset)begin
            enlargedReset <= 1;
            counter <= 0;
        end else begin
            if(counter > nOfDelays)begin
                enlargedReset <= 0;
            end else begin
                counter <= counter + 1;
                enlargedReset <= 1;
            end
        end
        
    end
    
    assign outReset = reset | enlargedReset; 

endmodule
