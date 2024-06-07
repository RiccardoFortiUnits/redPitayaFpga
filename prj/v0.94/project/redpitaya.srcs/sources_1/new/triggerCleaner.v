`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.06.2024 16:09:04
// Design Name: 
// Module Name: triggerCleaner
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


module triggerCleaner#(
    parameter nOfHinibitionCycles = 125//1e-6s
)(
    input clk,
    input reset,
    input in,
    input out
);
//reg in_r;
reg [$clog2(nOfHinibitionCycles+1)-1:0] hinibitionCounter;
//reg outToggled;
//assign out = in_r & outToggled & (!hinibitionCounter);
//always @(posedge clk)begin
//    if(reset)begin
//        hinibitionCounter <= 0;
//        outToggled <= 1;
//        in_r <= 0;
//    end else begin
//        in_r <= in;
//        if(in_r & outToggled & (!hinibitionCounter))begin
//            hinibitionCounter <= nOfHinibitionCycles;
//            outToggled <= 0;
//        end else begin
//            if(hinibitionCounter)begin
//                hinibitionCounter <= hinibitionCounter - 1;
//            end
//            outToggled <= outToggled | !in;
//        end
//    end
//end
localparam  s_idle = 0,
            s_active = 1,
            s_inhibit = 2;
reg [1:0] state;         // State variable (0: idle, 1: active, 2: inhibit)
reg out_r;     // Register for trigger output

always @(posedge clk) begin
    if(reset)begin
        hinibitionCounter <= 0;
        state <= s_idle;
        out_r <= 0;
    end else begin
        case (state)
            s_idle: begin
                // Idle state
                if (in) begin
                    state <= s_active;  // Transition to active state
                    out_r <= 1'b1;
                end
            end
            s_active: begin
                // Active state
                out_r <= 1'b0;
                state <= s_inhibit;      // Transition to inhibit state
                hinibitionCounter <= nOfHinibitionCycles;
            end
            s_inhibit: begin
                // Inhibit state
                out_r <= 1'b0;
                if(hinibitionCounter)begin
                    hinibitionCounter <= hinibitionCounter - 1;
                end else if (!in)begin
                    state <= s_idle;  // Transition back to idle state
                end
            end
        endcase
    end
end
assign out = out_r;
    
    
endmodule
