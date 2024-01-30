`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 30.01.2024 10:15:17
// Design Name: 
// Module Name: offsetSetter
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

module offsetSetter#(
    parameter     totalBits = 14,
    parameter     fracBits = 0,
    parameter     totalBits_extVoltageSignal = 14,
    parameter diodeResponsivity = 0.2,
    parameter tiagain1 = 20000,
    parameter tiagain2 = 4.9
)(
    input clk,
    input reset,
    input signed [totalBits-1:0] inputPower,
    output reg signed [totalBits-1:0] pidSetPoint,
    output reg signed [totalBits_extVoltageSignal-1:0] signalForExternalVoltage //it is considered as a positive value, 
                                                    //even though in the circuit it's always a negative tension
);
    function integer convertToFixedPoint(input real value, input integer fracBits);
        convertToFixedPoint = $rtoi(value * (1 << fracBits));
    endfunction
    wire [31:0] powerToOffsetCoefficient = convertToFixedPoint(diodeResponsivity*tiagain1*tiagain2 /(tiagain2+1), 13);
    wire [31:0] totalPowerGain =           convertToFixedPoint(diodeResponsivity*tiagain1*tiagain2,               13);
    wire [31:0] tiagain2_plus1 =           convertToFixedPoint(                                      tiagain2+1,  13);
    
    reg [totalBits-1:0] in;
    
    
    wire [totalBits-1:0] requiredExternalVoltage;
    wire [totalBits-1:0] realExternalVoltage;
    wire [totalBits_extVoltageSignal-1:0] obtainedSignalForExternalVoltage;
    wire [totalBits-1:0] finalInputVoltage;
    wire [totalBits-1:0] finalInputVoltage_delayed;
    wire [totalBits-1:0] finalOffsetVoltage;
    
    clocked_FractionalMultiplier#(totalBits,totalBits,totalBits,fracBits,fracBits,fracBits)
    powerToOffset                   (clk, in, powerToOffsetCoefficient, requiredExternalVoltage);
    
    clocked_FractionalMultiplier#(totalBits,totalBits,totalBits,fracBits,fracBits,fracBits)
    powerTofinalVoltage             (clk, in, totalPowerGain, finalInputVoltage);
    
    clocked_FractionalMultiplier#(totalBits,totalBits,totalBits,fracBits,fracBits,fracBits)
    offsetTofinalVoltage            (clk, realExternalVoltage, tiagain2_plus1, finalOffsetVoltage);
    
    delay#(2,totalBits) 
    delayFinalInputVoltage          (clk, finalInputVoltage, finalInputVoltage_delayed);
    
    requiredToRealOffset#(totalBits, fracBits, totalBits_extVoltageSignal)
    requiredToReal                  (clk, requiredExternalVoltage, realExternalVoltage, obtainedSignalForExternalVoltage);

    //todo we should also reset the PID, or the integral part would be incorrect in the new offset. 
        //Or does the software already do a reset?
    
    always @(posedge clk)begin
        if(reset)begin
            in <= 0;
            pidSetPoint <= 0;
        end else begin
            in <= inputPower;
            pidSetPoint <= finalOffsetVoltage - finalInputVoltage_delayed;
            signalForExternalVoltage <= obtainedSignalForExternalVoltage;
        end
    end
    
endmodule


module requiredToRealOffset#(
    parameter     totalBits = 14,
    parameter     fracBits = 0,
    parameter     totalBits_extVoltageSignal = 14
)(
    input clk,
    input [totalBits-1:0] requiredOffset,
    output reg [totalBits-1:0] realOffset,
    output reg [totalBits_extVoltageSignal-1:0] signalForExternalVoltage
);
    always @(posedge clk)begin
        
        //todo: do the actual conversion
        
    
        realOffset <= requiredOffset;
        signalForExternalVoltage <= requiredOffset;
        
    end
endmodule
