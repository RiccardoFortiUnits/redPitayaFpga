/**
 * $Id: red_pitaya_ams.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya analog mixed signal.
 *
 * @Author Matej Oblak
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in Verilog hardware description language (HDL).
 * Please visit http://en.wikipedia.org/wiki/Verilog
 * for more details on the language used herein.
 */

/**
 * GENERAL DESCRIPTION:
 *
 * Module using XADC and software interface for PWM DAC.
 *
 *
 *                    /------\
 *   SUPPLY V. -----> |      |
 *   TEMPERATURE ---> | XADC | ------
 *   EXTERNAL V. ---> |      |       |
 *                    \------/       |
 *                                   |
 *                                   ˇ
 *                               /------\
 *   PWD DAC <------------------ | REGS | <------> SW
 *                               \------/
 *
 *
 * Reading system and external voltages is done with XADC, running in sequencer
 * mode. It measures supply voltages, temperature and voltages on external
 * connector. Measured values are then exposed to SW.
 *
 * Beside that SW can sets registes which controls logic for PWM DAC (analog module).
 * 
 */

module red_pitaya_ams#(
    parameter pwm_size = 8
) (
    // ADC
    input                 clk_i           ,  // clock
    input                 rstn_i          ,  // reset - active low
    // PWM DAC
    output[pwm_size-1: 0] dac_a_o         ,  // values used for
    output[pwm_size-1: 0] dac_b_o         ,  // conversion into PWM signal
    output[pwm_size-1: 0] dac_c_o         ,  // 
    output[pwm_size-1: 0] dac_d_o         ,  // 
    // external signal inputs
    input      [ 14-1: 0] dat_a_i         ,  //!< input data CHA
    input      [ 14-1: 0] dat_b_i         ,  //!< input data CHB
    // system bus
    input      [ 32-1: 0] sys_addr        ,  // bus address
    input      [ 32-1: 0] sys_wdata       ,  // bus write data
    input                 sys_wen         ,  // bus write enable
    input                 sys_ren         ,  // bus read enable
    output reg [ 32-1: 0] sys_rdata       ,  // bus read data
    output reg            sys_err         ,  // bus error indicator
    output reg            sys_ack            // bus acknowledge signal
);
//new feature: you can control the value of the PWM outputs with the analog inputs, and you can 
    //even shift and resize their value for better mapping.
    //set the channel configuration to either 0x10 or 0x11 to select one of the two analog channels
    //keep it at 0 to use the value read from RAM 
    //for mapping, you can select the minimum value of the analog signal (any value lower or equal 
    //to that will result in an output = 0) and a multiplication factor

localparam scalerSize = 14;
localparam scalerFracBits = 10;
localparam timerSize = 24;

localparam  use_memory = 0,
            use_ramp   = 1,
            use_adc    = 2;
reg [1:0] conf_outputSelect[3:0];
reg conf_AdcSelect[3:0];
wire [13:0] selectedADC_input[3:0];

//set output from memory
reg [pwm_size-1:0] valuesFromMemory[3:0];

//set output from ramp
localparam  trigger_none   = 0,
            trigger_now    = 1,
            trigger_adc   = 2;
localparam  edge_pos    = 0,
            edge_neg    = 1;
reg [pwm_size-1:0] ramp_start[3:0];
reg [pwm_size-1:0] ramp_valueIncrementer[3:0];
reg [timerSize-1:0] ramp_timeIncremeter[3:0];
reg [pwm_size-1:0] ramp_nOfCycles[3:0];
reg [1:0] ramp_triggerSelect[3:0];
reg [13:0] ramp_ADCtriggerValue[3:0];
reg ramp_ADCtriggerEdge[3:0];
reg [1:0] ramp_idleConfig[3:0];
wire ramp_trigger[3:0];
wire ramp_ADCtrigger[3:0];
wire [pwm_size-1:0] ramp_Output[3:0];

//set output from ADC
reg [scalerSize-1:0] adc_scaler[3:0];
reg [13:0] adc_minValue[3:0];
wire [pwm_size-1:0] adc_conditionedOut[3:0];

reg [pwm_size-1:0] outs[3:0];
integer i;

generate
genvar gi;

for(gi = 0; gi < 4; gi = gi + 1)begin
    
    //select input ADC
    assign selectedADC_input[gi] = conf_AdcSelect[gi] == 0 ?
                                    dat_a_i :
                                  conf_AdcSelect[gi] == 1 ?
                                    dat_b_i :
                                    0;//should never happen
    
    //ramp
    assign ramp_ADCtrigger[gi] = ramp_ADCtriggerEdge[gi] == edge_pos ?//todo sarebbe da fare un trigger di un solo colpo di clock, oppure aggiungi alla rampa uno stato in cui aspetta che il trigger si abbassi
                                    $signed(selectedADC_input[gi]) >= $signed(ramp_ADCtriggerValue[gi]) : 
                                    $signed(selectedADC_input[gi]) <= $signed(ramp_ADCtriggerValue[gi]);
    
    assign ramp_trigger[gi] = ramp_triggerSelect[gi] == trigger_none ?
                                 0 :
                              ramp_triggerSelect[gi] == trigger_now ?
                                 1 :
                           // ramp_triggerSelect[gi] == trigger_adc
                                 ramp_ADCtrigger[gi];
    
    //ramp setup    
    ramp#(
        .data_size(pwm_size),
        .time_size(timerSize)
    )rrrrrrrr(
        .clk			(clk_i),
        .reset			(!rstn_i),
        .trigger		(ramp_trigger[gi]),
        .startPoint     (ramp_start[gi]),
        .stepIncrease   (ramp_valueIncrementer[gi]),
        .timeStep       (ramp_timeIncremeter[gi]),
        .nOfSteps       (ramp_nOfCycles[gi]),
        .idleConfig		(ramp_idleConfig[gi]),
        .out			(ramp_Output[gi])
    );
    
    //ADC conditioning
    shift_n_scale#(
        .input_size     (14),
        .output_size    (8),
        .scaler_size    (scalerSize),
        .scaler_fracBits(scalerFracBits)
    )sns(
        .clk          (clk_i),
        .reset        (!rstn_i),
        .in           (selectedADC_input[gi]),
        .out          (adc_conditionedOut[gi]),
        .minInputValue(adc_minValue[gi]),
        .scalingFactor(adc_scaler[gi])
    );
    
    
              
end

endgenerate

//select output
always @(posedge clk_i) begin
    if (!rstn_i) begin
        for(i = 0; i < 4; i=i+1)begin
            outs[i] <= 0;
        end
    end else begin
        for(i = 0; i < 4; i=i+1)begin
            outs[i] <= conf_outputSelect[i] == use_memory ? 
                           valuesFromMemory[i] :
                       conf_outputSelect[i] == use_ramp ?
                           ramp_Output[i] :
                       conf_outputSelect[i] == use_adc ?
                           adc_conditionedOut[i] :
                           0;
        end
    end
end
assign dac_a_o = outs[0];
assign dac_b_o = outs[1];
assign dac_c_o = outs[2];
assign dac_d_o = outs[3];

//read from memory
always @(posedge clk_i) begin
    if (!rstn_i) begin
        for(i = 0; i < 4; i=i+1)begin
            conf_outputSelect[i] <= use_memory;
            conf_AdcSelect[i] <= 0;
        
            ramp_start[i] <= 0;
            ramp_valueIncrementer[i] <= 0;
            ramp_timeIncremeter[i] <= 0;
            ramp_nOfCycles[i] <= 0;
            ramp_triggerSelect[i] <= 0;
            ramp_ADCtriggerValue[i] <= 0;
            ramp_ADCtriggerEdge[i] <= 0;
            ramp_idleConfig[i] <= 0;
            
            valuesFromMemory[i] <= 0;
            adc_minValue[i] <= 'h2000;
            adc_scaler[i] <= 1<<(scalerFracBits-1);
        end
    end else begin
        for(i = 0; i < 4; i=i+1)begin
            if (sys_wen) begin
                if (sys_addr[19:0] == 'h20 + (i << 2))begin//addresses 0x20, 0x24, 0x28, 0x2C
                    valuesFromMemory[i] <= sys_wdata[23:16];
                    conf_outputSelect[i] <= sys_wdata[1:0];
                    conf_AdcSelect[i] <= sys_wdata[2];
                end
                if (sys_addr[19:0] == 'h30 + (i << 2))begin//addresses 0x30, 0x34, 0x38, 0x3C
                    adc_minValue[i] <= sys_wdata[13:0];
                    adc_scaler[i] <= sys_wdata[14+scalerSize-1:14];
                end
                
                if (sys_addr[19:0] == 'h40 + (i << 2))begin//addresses 0x40, 0x44, 0x48, 0x4C
                    ramp_start[i] <= sys_wdata[pwm_size-1:0];
                    ramp_valueIncrementer[i] <= sys_wdata[pwm_size+pwm_size-1:pwm_size];
                end
                
                if (sys_addr[19:0] == 'h50 + (i << 2))begin//addresses 0x50, 0x54, 0x58, 0x5C
                    ramp_timeIncremeter[i] <= sys_wdata[timerSize-1:0];
                    ramp_nOfCycles[i] <= sys_wdata[timerSize+pwm_size-1:timerSize];
                end
                
                if (sys_addr[19:0] == 'h60 + (i << 2))begin//addresses 0x60, 0x64, 0x68, 0x6C
                    ramp_triggerSelect[i] <= sys_wdata[1:0];
                    ramp_idleConfig[i] <= sys_wdata[3:2];
                    ramp_ADCtriggerValue[i] <= sys_wdata[17:4];
                    ramp_ADCtriggerEdge[i] <= sys_wdata[18];
                end
                
            end else begin
                if(ramp_triggerSelect[i] == trigger_now)begin
                    ramp_triggerSelect[i] <= trigger_none;
                end     
            end
        end
    end
end
wire sys_en;
assign sys_en = sys_wen | sys_ren;

//write to memory
always @(posedge clk_i)begin
    if (rstn_i == 1'b0) begin
        sys_err <= 1'b0 ;
        sys_ack <= 1'b0 ;
    end else begin
        sys_err <= 1'b0 ;
        sys_ack <= sys_en;
        casez (sys_addr[19:0])    
            20'h20 : begin sys_rdata <= {valuesFromMemory[0], {16-3{1'b0}}, conf_AdcSelect[0], conf_outputSelect[0]}; end
            20'h24 : begin sys_rdata <= {valuesFromMemory[1], {16-3{1'b0}}, conf_AdcSelect[1], conf_outputSelect[1]}; end
            20'h28 : begin sys_rdata <= {valuesFromMemory[2], {16-3{1'b0}}, conf_AdcSelect[2], conf_outputSelect[2]}; end
            20'h2C : begin sys_rdata <= {valuesFromMemory[3], {16-3{1'b0}}, conf_AdcSelect[3], conf_outputSelect[3]}; end
            20'h30 : begin sys_rdata <= {adc_scaler[0], adc_minValue[0]}; end
            20'h34 : begin sys_rdata <= {adc_scaler[1], adc_minValue[1]}; end
            20'h38 : begin sys_rdata <= {adc_scaler[2], adc_minValue[2]}; end
            20'h3C : begin sys_rdata <= {adc_scaler[3], adc_minValue[3]}; end
            
            20'h40 : begin sys_rdata <= {ramp_valueIncrementer[0], ramp_start[0]}; end
            20'h44 : begin sys_rdata <= {ramp_valueIncrementer[1], ramp_start[1]}; end
            20'h48 : begin sys_rdata <= {ramp_valueIncrementer[2], ramp_start[2]}; end
            20'h4C : begin sys_rdata <= {ramp_valueIncrementer[3], ramp_start[3]}; end
            20'h50 : begin sys_rdata <= {ramp_nOfCycles[0], ramp_timeIncremeter[0]}; end
            20'h54 : begin sys_rdata <= {ramp_nOfCycles[1], ramp_timeIncremeter[1]}; end
            20'h58 : begin sys_rdata <= {ramp_nOfCycles[2], ramp_timeIncremeter[2]}; end
            20'h5C : begin sys_rdata <= {ramp_nOfCycles[3], ramp_timeIncremeter[3]}; end
            20'h60 : begin sys_rdata <= {ramp_ADCtriggerEdge[0], ramp_ADCtriggerValue[0], ramp_idleConfig[0], ramp_triggerSelect[0]}; end
            20'h64 : begin sys_rdata <= {ramp_ADCtriggerEdge[1], ramp_ADCtriggerValue[1], ramp_idleConfig[1], ramp_triggerSelect[1]}; end
            20'h68 : begin sys_rdata <= {ramp_ADCtriggerEdge[2], ramp_ADCtriggerValue[2], ramp_idleConfig[2], ramp_triggerSelect[2]}; end
            20'h6C : begin sys_rdata <= {ramp_ADCtriggerEdge[3], ramp_ADCtriggerValue[3], ramp_idleConfig[3], ramp_triggerSelect[3]}; end
            default : begin sys_rdata <=32'h0; end
        endcase
    end
end

endmodule
