`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.12.2023 12:19:28
// Design Name: 
// Module Name: movingAverage
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


module ir_filter(
    input clk_i,
    input [13:0] in,
    output [13:0] out
    );
    
    parameter prevVal_totalBits = 20;
    parameter prevVal_fracBits = prevVal_totalBits - 14;
    
    // Function to convert floating-point to fixed-point
    function signed [prevVal_totalBits-1:0] convertToFixedPoint(real value, integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    
    reg signed [prevVal_totalBits-1:0] y_delayed = 0; // Delayed output for feedback
    reg signed [prevVal_totalBits-1:0] a = convertToFixedPoint(0.8, prevVal_fracBits); // Example 'a' with 4 decimal bits
    reg signed [prevVal_totalBits-1:0] b = convertToFixedPoint(0.2, prevVal_fracBits); // Example 'b' with 4 decimal bits

    wire [prevVal_totalBits-1:0] ay_1;
    wire [prevVal_totalBits-1:0] bx;
    
    FractionalMultiplier 
        #(prevVal_totalBits,14,prevVal_totalBits,prevVal_fracBits,0,prevVal_fracBits) 
    ay_1_m (.a(a),.b(y_delayed),.result(ay_1));
    
    FractionalMultiplier 
        #(prevVal_totalBits,14,prevVal_totalBits,prevVal_fracBits,0,prevVal_fracBits) 
    bx_m (.a(b),.b(in),.result(bx));

    always @(posedge clk_i) begin
        y_delayed <= ay_1 + bx;
    end
  
  assign out = y_delayed[13+prevVal_fracBits:prevVal_fracBits];
//  assign out = in;
    
endmodule
