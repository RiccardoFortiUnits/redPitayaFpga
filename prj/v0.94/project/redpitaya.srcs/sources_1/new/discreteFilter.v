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
        input  [7:0] denNumSplit//should be higher than 0 (you need at least one coefficient that uses the input)
    );
    
    //implementation of a generic filter in the form y[n] = b0x[n]+b1x[n-1]+...bk[n-k] + a1y[n-1]+...ahy[n-h], or 
    // Y = BX + AY
    //since it is optimized for applicability and not for speed/space, this filter requires too many steps per cycle to function. 
    //Thus, I've added a buffer register in the middle of the calculations, that will inevitably modify the expected response 
    //(actual implemented filter: Y = z^-1*(BX+AY)), but at least it won't glitch out. If you want a real filter realization, 
    //try a pipelined block scheme that suits your specific filter.
    
    //the values for b0,b1,...a1,a2... are written in the array coefficients, and the number denNumSplit tells the module where 
    //the value a1 is (ex: denNumSplit == 2 => coefficients[0:] = [b0,b1,a1,a2,a3...]
    
    reg [workingBits-1:0] ins [max_nOfCoefficients-2:0];//only the previous values of the input, not the current one
    reg [workingBits-1:0] outs [max_nOfCoefficients-1:0];
    wire [workingBits-1:0] valsToMultiply [max_nOfCoefficients-1:0];
    
    wire [workingBits-1:0] multipliedVals [max_nOfCoefficients-1:0];
    reg [workingBits-1:0] multipliedRegs [max_nOfCoefficients-1:0];
    wire signed [workingBits-1:0] sums [(max_nOfCoefficients << 1)-2:0];
    wire [workingBits-1:0] saturated_sum;
        
generate
    genvar gi,j;
        
    //multipliers (coefficient * in/out)
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
        .result(multipliedVals[gi])
        );
    end
    
    //initialize the first elements of sums with the results of the multiplications
    for (gi = 0; gi < max_nOfCoefficients; gi = gi + 1) begin
        assign sums[gi] = multipliedRegs[gi];
    end
    
    //sums of every multiplicator. Optimized to have the higher parallelization possible (number of sums "in series" = log2(max_nOfCoefficients) )
    for(j=max_nOfCoefficients;j>0;j=j>>1)begin
        for (gi = 0; gi < (j>>1); gi = gi + 1) begin
            assign sums[(max_nOfCoefficients<<1)-j+gi] = sums[(max_nOfCoefficients<<1)-j-(gi<<1) - 2] + sums[(max_nOfCoefficients<<1)-j-(gi<<1) - 1];
        end
    end
    
    //in/out selectors
    assign valsToMultiply[0] = in;
    for (gi = 1; gi < max_nOfCoefficients; gi = gi + 1) begin
        assign valsToMultiply[gi] = (gi < denNumSplit) ? ins[gi-1] : outs[gi - denNumSplit];
    end
    
endgenerate

saturator#(workingBits, totalBits_IO) outSaturator(sums[(max_nOfCoefficients << 1)-2], saturated_sum);
    
    integer i;
    
    always @(posedge clk) begin
        if(reset)begin
            for(i=0;i<max_nOfCoefficients;i=i+1)begin
                ins[i] <= 0;
                outs[i] <= 0;
                multipliedRegs[i] <= 0;
            end
        end else begin                
            
            for(i=1;i<max_nOfCoefficients-1;i=i+1)begin
                ins[i] <= ins[i-1];
            end
            for(i=1;i<max_nOfCoefficients;i=i+1)begin
                outs[i] <= outs[i-1];
            end
            for(i=0;i<max_nOfCoefficients;i=i+1)begin
                multipliedRegs[i] <= multipliedVals[i];
            end
            
            ins[0] <= in;
            outs[0] <= saturated_sum;
        end 
    end

assign out = outs[0];
    
endmodule
