
module ir_filter_fixed#(
    parameter cutoffFrequency = 0.5,
    parameter totalBits = 30,
    parameter fracBits = totalBits - 14
)(
    input clk_i,
    input [13:0] in,
    output [13:0] out
    );
    
  
    // Function to convert floating-point to fixed-point
    function signed [totalBits-1:0] convertToFixedPoint(real value, integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    
    localparam real pi = 3.141592654;
    function real a_from_fcT(real fcT);
        a_from_fcT = (1 - $cos(2*pi*fcT)) / (1 - $cos(2*pi*fcT) + $sin(2*pi*fcT));
    endfunction
    
    reg [totalBits-1:0] coefficient;
    
    initial begin
        coefficient = convertToFixedPoint(a_from_fcT(cutoffFrequency), fracBits);
    end
    
    ir_filter#(totalBits, fracBits) irf(
    clk_i,
    in,
    coefficient,
    out
    );

endmodule