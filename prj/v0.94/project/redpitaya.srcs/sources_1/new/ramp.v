`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01.03.2024 16:13:05
// Design Name: 
// Module Name: ramp
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



module ramp#(
	parameter data_size = 16,
	parameter time_size = 16
)(
	input clk,
	input reset,
	input trigger,
	input signed [data_size-1:0] startPoint,
	input signed [data_size-1:0] stepIncrease,
	input [time_size-1:0] timeStep,
	input [data_size-1:0] nOfSteps,//since the output is incremented every cycle, 
	                           //we should not have more cycles than possible output values
	input [1:0] idleConfig,
	output reg [data_size-1:0] out
);
// generates a ramp. When a trigger is detected, it sets the output to the value of startPoint, and 
	//it starts a counter. Every time the counter reaches the value of timeStep, the output gets increased
	//by the value of stepIncrease. When nOfSteps cycles have been executed, the ramp stops
	
//if you start from the ramp paramters startValue, endValue and rampTime, yo

//let's save all the parameters in some registers that will keep the value for the duration of the ramp (is it really necessary?)
reg [data_size-1:0] startPoint_r;
reg [time_size-1:0] timeStep_r;
reg [data_size-1:0] stepIncrease_r;
reg [1:0] idleConfig_r;


//states
localparam 	s_idle = 0,
			s_running = 1;
reg [0:0] state;

//configuration of idle state: on which value do we stay while the module is waiting for a trigger?
localparam 	c_zero = 0,		
			c_start = 1,
			c_current = 2;

//counters
reg [time_size-1:0] stepCounter;
reg [data_size-1:0] cycleCounter;

always @(posedge clk)begin
	if(reset)begin
		state <= s_idle;

		stepCounter <= 0;
		cycleCounter <= 0;

		startPoint_r <= 0;
		timeStep_r <= 0;
		stepIncrease_r <= 0;
		idleConfig_r <= 0;

	end else begin
		case(state)
			s_idle: begin
				if(trigger)begin
					state <= s_running;

					//let's start the counters
					stepCounter <= timeStep - 1;
					cycleCounter <= nOfSteps - 1;

					//fix the values of the input parameters
					startPoint_r <= startPoint;
					timeStep_r <= timeStep - 1;
					stepIncrease_r <= stepIncrease;
					idleConfig_r <= idleConfig;

					//set the first value of the output
					out <= startPoint;
				end
			end
			s_running: begin
				if(!stepCounter)begin//one cycle finished?
					if(!cycleCounter)begin//final cycle finished?
						state <= s_idle;						
                        //set idle output
                        case(idleConfig)
                            c_zero: begin		out <= 0; 			end
                            c_start: begin		out <= startPoint_r;end
                            //c_current: begin	out <= out; 		end
                            default: begin end
                        endcase						
					end else begin
						stepCounter <= timeStep_r;//reset step counter
						cycleCounter <= cycleCounter - 1;//reduce cycle counter
						out <= out + stepIncrease_r;//go to the next ramp value
					end
				end else begin
					//reduce step counter
					stepCounter <= stepCounter - 1;
				end
			end
		endcase
	end
end

endmodule