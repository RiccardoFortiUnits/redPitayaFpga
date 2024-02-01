
module ir_filter_fixed#(
    parameter cutoffFrequency = 0.5,
    parameter totalBits_IO = 14   ,
    parameter fracBits_IO = 0     ,
    parameter totalBits_coeff = 30,
    parameter fracBits_coeff = totalBits_coeff - 14
)(
    input clk_i,
    input reset,
    input [totalBits_IO-1:0] in,
    output [totalBits_IO-1:0] out
    );
    function integer convertToFixedPoint(input real value, input integer fracBits);
    convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    
    localparam real pi = 3.141592654;
    function real a_from_fcT(input real fcT);
        a_from_fcT = (1 - $cos(2*pi*fcT)) / (1 - $cos(2*pi*fcT) + $sin(2*pi*fcT));
    endfunction
    
    reg [totalBits_coeff-1:0] coefficient;
    
    initial begin
        coefficient = convertToFixedPoint(a_from_fcT(cutoffFrequency), fracBits_coeff);
    end
    
    ir_filter#(totalBits_IO, fracBits_IO, totalBits_coeff, fracBits_coeff) irf(
    clk_i,
    reset,
    in,
    coefficient,
    out
    );

endmodule