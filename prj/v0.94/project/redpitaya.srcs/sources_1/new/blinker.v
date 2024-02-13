`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06.02.2024 10:17:30
// Design Name: 
// Module Name: blinker
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


module blinker#(
    parameter blinkClockCycles = 12500000
)(
    input clk,
    input enable,
    input reset,
    output reg out
);
    //toggles the exit with a specified period, as long as it sees at least one enable signal in each period. 
        //When no enable is received, it will finish the current period and then turn off the output   
    
    localparam nOfBits = $clog2(blinkClockCycles);
    
    localparam  idle        = 0,
                on_high     = 1,
                turningOff  = 2,
                on_low      = 3;
    
    reg [1:0] state;
    
    reg [nOfBits:0] counter;
    
    always @(posedge clk)begin
        if(reset) begin
            out <= 0;
            counter <= 0;
            state <= idle;
        end else begin            
            casez (state)
                idle        : begin 
                    if(enable) begin
                        //let's start the toggling
                        state <= on_high;
                        counter <= blinkClockCycles;
                    end
                end
                
                on_high     : begin
                    if(!counter) begin
                        //high half-period done
                        state <= turningOff;
                        counter <= blinkClockCycles;
                    end else begin
                        counter <= counter - 1;
                    end
                end
                
                turningOff  : begin
                    if(!counter) begin
                        //we haven't received any new enable, we should not start the new blink
                        state <= idle;
                    end else begin
                        //let's wait to finish the low half-period before starting the new blink
                        counter <= counter - 1;
                        if(enable) begin
                            state <= on_low;
                        end
                    end
                end
                
                on_low      : begin 
                    if(!counter) begin
                        //low half-period done
                        state <= on_high;
                        counter <= blinkClockCycles;
                    end else begin
                        counter <= counter - 1;
                    end 
                end
            endcase
            
            out <= state == on_high;
        end
    end
    
endmodule
