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


module ir_filter#(//the parameters for IO should be smaller than the parameters of the coefficient:
    //totalBits_IO < totalBits_coeff,
    //fracBits_IO < fracBits_coeff,
    //totalBits_IO - fracBits_IO < totalBits_coeff - fracBits_coeff
    parameter totalBits_IO = 14   ,
    parameter fracBits_IO = 0     ,
    parameter totalBits_coeff = 30,
    parameter fracBits_coeff = totalBits_coeff - 14
)(
    input clk_i,
    input reset,
    input [totalBits_IO-1:0] in,
    input [totalBits_coeff-1:0] coefficient,
    output [totalBits_IO-1:0] out
    );
    
    
    // Function to convert floating-point to fixed-point
    function signed [totalBits_coeff-1:0] convertToFixedPoint(real value, integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    
    reg signed [totalBits_coeff-1:0] y_delayed; // Delayed output for feedback
    wire signed [totalBits_coeff-1:0] a = coefficient;//convertToFixedPoint(0.9999, fracBits_coeff);
    wire signed [totalBits_coeff-1:0] b = convertToFixedPoint(1, fracBits_coeff) - a;

    wire [totalBits_coeff-1:0] ay_1;
    wire [totalBits_coeff-1:0] bx;
    
    FractionalMultiplier 
        #(totalBits_coeff,totalBits_coeff,totalBits_coeff,fracBits_coeff,fracBits_coeff,fracBits_coeff) 
    ay_1_m (.a(a),.b(y_delayed),.result(ay_1));
    
    FractionalMultiplier 
        #(totalBits_coeff,totalBits_IO,totalBits_coeff,fracBits_coeff,fracBits_IO,fracBits_coeff) 
    bx_m (.a(b),.b(in),.result(bx));

    always @(posedge clk_i) begin
        if(reset)
            y_delayed = 0;
        else
            y_delayed <= ay_1 + bx;
    end
  
  assign out = y_delayed[fracBits_coeff - fracBits_IO + totalBits_IO - 1 : fracBits_coeff - fracBits_IO];
//  assign out = in;
    
endmodule
