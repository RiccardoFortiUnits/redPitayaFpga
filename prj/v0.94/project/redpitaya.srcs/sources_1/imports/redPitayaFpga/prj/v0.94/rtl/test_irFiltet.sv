`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.12.2023 14:09:20
// Design Name: 
// Module Name: test_irFiltet
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


module test_irFiltet(

    );
    
    reg [13:0] x;
    assign x = 'h0AC4;
    wire [13:0] y;
    
    localparam CLK_PERIOD = 10;
    
    reg clk;
    initial clk = 0;
    always #(CLK_PERIOD/2.0)
        clk = ~clk;
    
    
ir_filter ma2(
    .clk_i           (clk   ),
    .in(x),
    .out(y)
);
    
endmodule
