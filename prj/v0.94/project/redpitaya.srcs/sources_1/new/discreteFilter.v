`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.01.2024 11:01:05
// Design Name: 
// Module Name: discreteFilter
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


module discreteFilter#(
        parameter     totalBits_IO = 14   ,
        parameter     fracBits_IO = 0     ,
        parameter     totalBits_coeffs = 24   ,
        parameter     fracBits_coeffs = 10    ,
        parameter     workingBits = 32    ,
        
        parameter     max_nOfCoefficients = 8//must be a power of 2
    )(
        input  clk,
        input  reset,
        input signed [totalBits_IO-1:0] in,
        output signed [totalBits_IO-1:0] out,
        
        input signed [max_nOfCoefficients-1:0][totalBits_coeffs-1:0] coefficients,
        input  [7:0] denNumSplit
    );
    
    reg [workingBits-1:0] ins [max_nOfCoefficients-1:0];
    reg [workingBits-1:0] outs [max_nOfCoefficients-1:0];
    wire [workingBits-1:0] valsToMultiply [max_nOfCoefficients-1:0];
    
    wire signed [workingBits-1:0] sums [(max_nOfCoefficients << 1)-2:0];
    wire [workingBits-1:0] saturated_sum;
    
generate
    genvar gi,j;
    for (gi = 0; gi < max_nOfCoefficients; gi = gi + 1) begin
        FractionalMultiplier #(
        .A_WIDTH(totalBits_coeffs),
        .B_WIDTH(workingBits),
        .OUTPUT_WIDTH(workingBits),
        .FRAC_BITS_A(fracBits_coeffs),
        .FRAC_BITS_B(fracBits_IO),
        .FRAC_BITS_OUT(fracBits_IO)
        ) mult_b (
        .a(coefficients[gi]),
        .b(valsToMultiply[gi]),
        .result(sums[gi])
        );
    end
    for(j=max_nOfCoefficients;j>0;j=j>>1)begin
        for (gi = 0; gi < (j>>1); gi = gi + 1) begin
            assign sums[(max_nOfCoefficients<<1)-j+gi] = sums[(max_nOfCoefficients<<1)-j-(gi<<1) - 2] + sums[(max_nOfCoefficients<<1)-j-(gi<<1) - 1];
        end
    end
    
    for (gi = 0; gi < max_nOfCoefficients; gi = gi + 1) begin
        assign valsToMultiply[gi] = (gi < denNumSplit) ? ins[gi] : outs[gi - denNumSplit];
    end
    
endgenerate

saturator#(workingBits, totalBits_IO) outSaturator(sums[(max_nOfCoefficients << 1)-2], saturated_sum);
    
    integer i;
    
    always @(posedge clk) begin
        if(reset)begin
            for(i=0;i<max_nOfCoefficients;i=i+1)begin
                ins[i] <= 0;
                outs[i] <= 0;
            end
        end else begin
                
            
            for(i=1;i<max_nOfCoefficients;i=i+1)begin
                ins[i] <= ins[i-1];
                outs[i] <= outs[i-1];
            end
            
            ins[0] <= in;
            outs[0] <= saturated_sum;
        end 
    end

assign out = outs[0];
    
endmodule
