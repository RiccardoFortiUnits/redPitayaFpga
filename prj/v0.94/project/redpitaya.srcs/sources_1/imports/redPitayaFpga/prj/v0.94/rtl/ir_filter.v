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
    
    parameter totalBits = 30;
    parameter fracBits = totalBits - 14;
    
    // Function to convert floating-point to fixed-point
    function signed [totalBits-1:0] convertToFixedPoint(real value, integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    
    reg signed [totalBits-1:0] y_delayed; // Delayed output for feedback
    reg signed [totalBits-1:0] a = convertToFixedPoint(0.9999, fracBits);
    reg signed [totalBits-1:0] b = convertToFixedPoint(0.0001, fracBits);

    initial begin
        y_delayed = 0;
    end

    wire [totalBits-1:0] ay_1;
    wire [totalBits-1:0] bx;
    
    FractionalMultiplier 
        #(totalBits,totalBits,totalBits,fracBits,fracBits,fracBits) 
    ay_1_m (.a(a),.b(y_delayed),.result(ay_1));
    
    FractionalMultiplier 
        #(totalBits,14,totalBits,fracBits,0,fracBits) 
    bx_m (.a(b),.b(in),.result(bx));

    always @(posedge clk_i) begin
        y_delayed <= ay_1 + bx;
    end
  
  assign out = y_delayed[13+fracBits:fracBits];
//  assign out = in;
    
endmodule
