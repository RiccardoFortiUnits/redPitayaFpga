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
    parameter nOf_ramps = 1,
	parameter data_size = 16,
	parameter time_size = 16,
	parameter hinibitionTimeForTrigger = 125//1e-6s
)(
	input clk,
	input reset,
	input trigger,
	input useMultipleTriggers,
	input signed [data_size-1:0] defaultValue,
	input        [$clog2(nOf_ramps):0] usedRamps,
	input signed [nOf_ramps-1:0][data_size-1:0] startPoint,
	input signed [nOf_ramps-1:0][data_size-1:0] stepIncrease, // = (endPoint - startPoint) / nOfSteps
	input        [nOf_ramps-1:0][time_size-1:0] timeStep, // = rampTime / nOfSteps
	input        [nOf_ramps-1:0][data_size-1:0] nOfSteps,
	input [1:0] idleConfig,
	output reg [data_size-1:0] out,
	output isBusy
);
// generates a ramp. When a trigger is detected, it sets the output to the value of startPoint, and 
	//it starts a counter. Every time the counter reaches the value of timeStep, the output gets increased
	//by the value of stepIncrease. When nOfSteps cycles have been executed, the ramp stops
	
//if you start from the ramp paramters startValue, endValue and rampTime, yo

//let's save all the parameters in some registers that will keep the value for the duration of the ramp (is it really necessary?)
reg [data_size-1:0] startPoint_r	[nOf_ramps-1:0];
reg [time_size-1:0] timeStep_r		[nOf_ramps-1:0];
reg [data_size-1:0] nOfSteps_r		[nOf_ramps-1:0];
reg [data_size-1:0] stepIncrease_r	[nOf_ramps-1:0];
reg [1:0] idleConfig_r;
reg [data_size-1:0] defaultValue_r;


//states
localparam 	s_idle = 0,
			s_running = 1,
			s_waitingForIntermediateTrigger = 2;
reg [1:0] state;
assign isBusy = state != s_idle;

//configuration of idle state: on which value do we stay while the module is waiting for a trigger?
localparam 	c_defaultValue = 0,		
			c_start = 1,
			c_current = 2,
			c_inverseRamp = 3;

//counters
reg [time_size-1:0] stepCounter;
reg [data_size-1:0] cycleCounter;
reg [$clog2(nOf_ramps):0] currentRamp;
reg [$clog2(nOf_ramps):0] usedRamps_r;

//trigger cleaner
wire cleanTrigger;
triggerCleaner#(
    .nOfHinibitionCycles(hinibitionTimeForTrigger)
)tc(
    .clk    (clk),
    .reset  (reset),
    .in     (trigger),
    .out    (cleanTrigger)
);

integer i;
reg rampIncreaser;//0: increase currentRamp, 1: decrease currentRamp (used for doing inverse ramp)
wire isLastRamp = (!rampIncreaser & ($unsigned(currentRamp) >= $unsigned(usedRamps_r - 1))) || (rampIncreaser & (currentRamp == 0));
always @(posedge clk)begin
    if(reset)begin
        state <= s_idle;
        stepCounter <= 0;
        cycleCounter <= 0;
        currentRamp <= 0;
        usedRamps_r <= 0;
        idleConfig_r <= 0;
        defaultValue_r <= 0;
        rampIncreaser <= 0;      
        for(i = 0; i < nOf_ramps; i = i + 1) begin
            startPoint_r [i]	<= 0;
            timeStep_r [i]		<= 0;
            nOfSteps_r [i]		<= 0;
            stepIncrease_r [i]	<= 0; 
        end
    end else begin
        case(state)
            s_idle: begin
                if(cleanTrigger && usedRamps)begin
                    state <= s_running;

                    //let's start the counters
                    stepCounter <= timeStep[0];
                    cycleCounter <= nOfSteps[0];
                    currentRamp <= 0;
                    usedRamps_r <= usedRamps;
                    rampIncreaser <= 0;

                    //fix the values of the input parameters
                    for(i = 0; i < nOf_ramps; i = i + 1) begin
                        startPoint_r[i] <= startPoint[i];
                        timeStep_r[i] <= timeStep[i];
                        nOfSteps_r[i] <= nOfSteps[i];
                        stepIncrease_r[i] <= stepIncrease[i];
                    end
                    idleConfig_r <= idleConfig;
                    defaultValue_r <= defaultValue;

                    //set the first value of the output
                    out <= startPoint[0];
                end
            end
            s_running: begin
                if((cleanTrigger && useMultipleTriggers) ||//should we already start the next ramp? 
                  $unsigned(stepCounter) < 2)begin//one cycle finished?                    
                    //(since we did an extra step during the cycle that transitioned between s_idle and 
                        //s_running (or between the end of a ramp to the start of the following one), 
                        //we should stop when the counter reaches 1 instead of 0. And, to avoid problems 
                        //when the timers are not initialized (default to 0), let's also consider the case in 
                        //which the counter is 0
                    if((cleanTrigger && useMultipleTriggers) ||//should we already start the next ramp? 
                      $unsigned(cycleCounter) < 2)begin//final cycle finished?
                        if(idleConfig_r == c_inverseRamp)begin//inverse ramp?
                            //reverse the step increaser, so that we'll move backwards
                            stepIncrease_r[currentRamp] <= - stepIncrease_r[currentRamp];
                            startPoint_r[currentRamp] <= out;//todo when using useMultipleTriggers and a trigger 
                                        //arrives early, we might not start from the actual end value... maybe we 
                                        //can reduce the number of steps, so that we'll end at the start value in any case
                        end
                        
                        if(isLastRamp)begin//final ramp finished?
                            if(idleConfig_r == c_inverseRamp)begin//inverse ramp?
                                //restart the ramps, but starting from the last one
                                rampIncreaser <= 1;
                                
                                stepCounter <= timeStep_r[currentRamp];//reset the counters
                                cycleCounter <= nOfSteps_r[currentRamp];
                            
                                idleConfig_r <= c_current;//remove the inverseRamp configuration, so that we won't repeat it again
                                
                                if(useMultipleTriggers && !cleanTrigger)begin
                                    state <= s_waitingForIntermediateTrigger;
                                end
                            end else begin
                                //set idle output
                                case(idleConfig_r)
                                    c_defaultValue: begin		out <= defaultValue;end
                                    c_start:        begin		out <= startPoint_r[0];end
                                    //c_current:    begin	    out <= out; 		end
                                    default: begin end
                                endcase
                                //reset the rest                                
                                state <= s_idle;
                                stepCounter <= 0;
                                cycleCounter <= 0;
                                currentRamp <= 0;
                                usedRamps_r <= 0;
                                idleConfig_r <= 0;
                                defaultValue_r <= 0;
                                rampIncreaser <= 0;      
                                for(i = 0; i < nOf_ramps; i = i + 1) begin
                                    startPoint_r [i]	<= 0;
                                    timeStep_r [i]		<= 0;
                                    nOfSteps_r [i]		<= 0;
                                    stepIncrease_r [i]	<= 0; 
                                end
                            end
                        end else begin
                            //go to the next ramp
                            if(!rampIncreaser)begin
                                currentRamp <= currentRamp + 1;
                                stepCounter <= timeStep_r[currentRamp + 1];//reset the counters
                                cycleCounter <= nOfSteps_r[currentRamp + 1];
                                out <= startPoint_r[currentRamp + 1];
                            end else begin
                                currentRamp <= currentRamp - 1;
                                stepCounter <= timeStep_r[currentRamp - 1];//reset the counters
                                cycleCounter <= nOfSteps_r[currentRamp - 1];
                                out <= startPoint_r[currentRamp - 1];
                            end
                            
                            if(useMultipleTriggers && !cleanTrigger)begin
                                state <= s_waitingForIntermediateTrigger;
                            end
                        end
                    end else begin
                        //reset counters for next cycle
                        stepCounter <= timeStep_r[currentRamp];//reset step counter
                        cycleCounter <= cycleCounter - 1;//reduce cycle counter
                        out <= out + stepIncrease_r[currentRamp];//go to the next ramp value
                    end
                end else begin
                    //reduce step counter
                    stepCounter <= stepCounter - 1;
                end
            end
            s_waitingForIntermediateTrigger: begin
                if(cleanTrigger)begin
                    state <= s_running;
                end
            end
        endcase
    end
end
endmodule






