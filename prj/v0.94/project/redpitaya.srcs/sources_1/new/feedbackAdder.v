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
    parameter cfg_noFeedback = 0,
              cfg_negFeedback = 1,
              cfg_posFeedback = 2,
              cfg_reverseOutput = 3;
    assign out = 
             (feedbackType == cfg_noFeedback)       ?   in : //no feedback
             (feedbackType == cfg_negFeedback)      ?   in - feedback : //negative feedback
             (feedbackType == cfg_posFeedback)      ?   in + feedback : //positive feedback
           /*(feedbackType == cfg_reverseOutput)    ?*/ - in;           //no feedback, output reversed
endmodule
