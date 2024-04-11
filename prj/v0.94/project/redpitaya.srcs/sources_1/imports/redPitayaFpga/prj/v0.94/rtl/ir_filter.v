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

//low-pass filter of the form y(n) = (1-a)x(n) + ay(n-1)

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
    function integer convertToFixedPoint(input real value, input integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    reg [totalBits_IO-1:0] in_reg;
    
    reg signed [totalBits_coeff-1:0] y_delayed; // Delayed output for feedback
    reg signed [totalBits_coeff-1:0] a;//set as a register because otherwise the timing analyser 
                    //might complain. It's not a very sensitive line, but still, it's better to protect its value with a register
    wire signed [totalBits_coeff-1:0] b = convertToFixedPoint(1, fracBits_coeff) - a;

    wire [totalBits_coeff-1:0] ay_1;
    wire [totalBits_coeff-1:0] bx;
    
    //ay_1 = a * y_delayed;
    FractionalMultiplier 
        #(totalBits_coeff,totalBits_coeff,totalBits_coeff,fracBits_coeff,fracBits_coeff,fracBits_coeff) 
    ay_1_m (.a(a),.b(y_delayed),.result(ay_1));
    
    //bx = b * in_reg
    FractionalMultiplier 
        #(totalBits_coeff,totalBits_IO,totalBits_coeff,fracBits_coeff,fracBits_IO,fracBits_coeff) 
    bx_m (.a(b),.b(in_reg),.result(bx));

    always @(posedge clk_i) begin
        if(reset)begin
            y_delayed <= 0;
            in_reg <= 0;
            a <= 0;
        end else begin
            y_delayed <= ay_1 + bx;
            in_reg = in;
            a <= coefficient;
        end
    end
  
  assign out = y_delayed[fracBits_coeff - fracBits_IO + totalBits_IO - 1 : fracBits_coeff - fracBits_IO];
//  assign out = in;
    
endmodule
