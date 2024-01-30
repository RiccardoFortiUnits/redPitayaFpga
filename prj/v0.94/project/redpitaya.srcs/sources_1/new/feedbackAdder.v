`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.01.2024 13:53:15
// Design Name: 
// Module Name: feedbackAdder
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

//depending on the value feedbackType, add or subtract the feedback signal to the input

module feedbackAdder#(
        parameter dataSize = 14
    )(
    input [1:0] feedbackType,
    input [dataSize-1:0] in,
    input [dataSize-1:0] feedback,
    output [dataSize-1:0] out
    );
//    always @(*)begin
//        case(feedbackType)
//            'b00: begin out <= in; end 
//            'b01: begin out <= in - feedback; end 
//            'b10: begin out <= in + feedback; end 
//            'b11: begin out <= feedback - in; end
//        endcase
//    end
    assign out = 
             (feedbackType == 'b00) ?   in :            //no feedback
             (feedbackType == 'b01) ?   in - feedback : //negative feedback
             (feedbackType == 'b10) ?   in + feedback : //positive feedback
           /*(feedbackType == 'b11) ?*/ - in;           //no feedback, output reversed
endmodule
