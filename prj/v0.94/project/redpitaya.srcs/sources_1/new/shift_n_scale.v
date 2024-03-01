`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.02.2024 12:06:44
// Design Name: 
// Module Name: shift_n_scale
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


module shift_n_scale#(
    parameter input_size = 14,// input is a number between -1 and 1 (fracBits = size - 1)
    parameter output_size = 8,// if unsigned, output is a number between 0 and 1. Otherwise it's between -1 and 1
    parameter scaler_size = 14,
    parameter scaler_fracBits = 10, 
    parameter output_isSigned = 0, //if signed, output_size also includes the sign bit
    parameter scaleFirst_shiftLater = 0 //switches the order of operations
)(
    input  clk,
    input  reset,
    input  [input_size-1:0] in,
    output [output_size-1:0] out,
    input  [input_size-1:0] minInputValue,
    input  [scaler_size-1:0] scalingFactor
);

reg [input_size-1:0] in_r, minInputValue_r;
reg [scaler_size-1:0] scalingFactor_r;

always @(posedge clk)begin
    if(reset)begin
        in_r <= 0;
        minInputValue_r <= 0;
        scalingFactor_r <= 0;
    end else begin
        in_r <= in;
        minInputValue_r <= minInputValue;
        scalingFactor_r <= scalingFactor;
    end
end

localparam productProtectionBits = 1 + scaler_size - scaler_fracBits;

wire [input_size-1+1:0] shiftedInput = {in_r[input_size-1],in_r} - {minInputValue_r[input_size-1],minInputValue_r};//let's add a bit 
                                                                                                                //to the subtraction
wire [output_size + productProtectionBits -1:0] out_toBeCropped, out_Cropped;

clocked_FractionalMultiplier
    #(input_size + 1, scaler_size    , output_size + productProtectionBits,
      input_size - 1, scaler_fracBits, output_size)
    mult                   (.clk(clk), .a(shiftedInput), .b(scalingFactor_r), .result(out_toBeCropped));

precisionSaturator
    #(output_size + productProtectionBits, (1 << output_size) - 1, 0)
    saturator               (.input_data(out_toBeCropped), .saturated_output(out_Cropped));

assign out = out_Cropped[output_size-1:0];

endmodule

















