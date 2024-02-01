`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.12.2023 16:02:31
// Design Name: 
// Module Name: delay_simulator
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


module delay_simulator#(
    parameter max_nOfCycles = 300,
    parameter dataSize = 14
)(
    input clk,
    input [dataSize-1:0] in,
    input [9:0] nOfDelays,// Adjust the range based on your maximum delay
    output [dataSize-1:0] out
    );
    reg [dataSize-1:0] delay_buffer [0:max_nOfCycles]; //(max_nOfCycles + 1) registers
    reg [9:0] nOfDelays_saturated;
    
    always @(*)begin
        nOfDelays_saturated <= (max_nOfCycles < nOfDelays) ? max_nOfCycles : nOfDelays;
    end
    
    integer i;
  
  always @(posedge clk) begin
  
      // Shift data through the delay buffer
      for (i = max_nOfCycles; i > 0; i = i - 1) begin
        delay_buffer[i] <= delay_buffer[i - 1];
      end
    
      // Store the current input in the delay buffer
      delay_buffer[0] <= in;
    
  end

  // Output the delayed data after N cycles
  assign out = delay_buffer[nOfDelays_saturated];
    
    
endmodule

module delay_s #(
    parameter nOfDelays = 1,
    parameter dataSize = 14
)(
    input clk,
    input [dataSize-1:0] in,
    output [dataSize-1:0] out
);

    localparam croppedNOfDelays = (nOfDelays > 0) ? nOfDelays-1 : 0;

    reg [dataSize-1:0] delayBuffers [croppedNOfDelays:0];

    integer i;

    always @(posedge clk) begin
        // Shift data through the delay buffer
        for (i = croppedNOfDelays; i > 0; i = i - 1) begin
            delayBuffers[i] <= delayBuffers[i - 1];
        end

        // Store the current input in the delay buffer
        delayBuffers[0] <= in;
    end

    // Output assignment
    assign out = (nOfDelays > 0) ? delayBuffers[croppedNOfDelays] : in;

endmodule