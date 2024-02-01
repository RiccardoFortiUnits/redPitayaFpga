
module PWM#(
    parameter     totalBits = 14,
    parameter     fracBits = 0,
    parameter     N_decimation = 100,
    parameter     filterInput = 0
)(
    input clk,
    input reset,
    input signed [totalBits-1:0] in,
    input signed [totalBits-1:0] minValue,
    input signed [totalBits-1:0] maxValue,
    output reg signed out
);

    function integer convertToFixedPoint(input real value, input integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    
    wire [totalBits-1:0]downsampledIn;
    wire downsampledClk;
    
    generate
        if(filterInput)begin
            //filter the input with a downsampler
            downsampler#(totalBits, fracBits, N_decimation)
            ds               (in, downsampledIn, clk, downsampledClk, reset);
        end else begin
            //no need to filter the input, we just need to divide the clock
            clockDivider#(N_decimation)
            cd               (clk, downsampledClk, reset);
            //and update the decimated input
            reg [totalBits-1:0]downsampledIn_reg;
            always @(posedge downsampledClk)begin
                downsampledIn_reg <= in;
            end
            assign downsampledIn = downsampledIn_reg;
        end
    endgenerate
    
    reg [totalBits-1:0] currentValue;
    reg [totalBits-1:0] incrementer;
    wire [totalBits-1:0] incrementerWire;
    
    FractionalMultiplier#(totalBits,totalBits,totalBits,fracBits,fracBits,fracBits)//todo if fracBits is not large enough, we might have problems
    calcIncrementer     (maxValue - minValue, convertToFixedPoint(1.0 / N_decimation,fracBits), incrementerWire);
    
    always @(posedge downsampledClk)begin
        currentValue <= minValue;
        incrementer <= incrementerWire;
    end
    
    always @(posedge clk)begin
        if(reset) begin
            currentValue <= 0;
            incrementer <= 0;
            out <= 0;
        end else begin
            currentValue <= currentValue + incrementer;
            out <= $signed(currentValue) < $signed(downsampledIn);
        end
    end

endmodule






