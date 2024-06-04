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
    parameter pwm_size = 8,
    parameter nOfDigitalPinsForTrigger = 16
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
    input[nOfDigitalPinsForTrigger-1:0] digitalInputs,
    // system bus
    input      [ 32-1: 0] sys_addr        ,  // bus address
    input      [ 32-1: 0] sys_wdata       ,  // bus write data
    input                 sys_wen         ,  // bus write enable
    input                 sys_ren         ,  // bus read enable
    output reg [ 32-1: 0] sys_rdata       ,  // bus read data
    output reg            sys_err         ,  // bus error indicator
    output reg            sys_ack            // bus acknowledge signal
);
//new feature: the value of the PWM outputs can be controlled with the analog inputs, or it can be set to create ramps
    //the input of the analog inputs can be shifted and multiplied before being converted to PWM, so that you can decide 
        //how to map the signal from the ADC range [-1V,+1V] to the PWM range [0,1.8V].
    //the ramp function can be triggered either from a RAM write command or an external trigger on th analog inputs
        //the ramp start point, end point, slope, and resolution can be modified
//the output can also be processed through a segmented function, which will render the output nonlinear.

localparam scalerSize = 14;
localparam scalerFracBits = 10;
localparam timerSize = 24;

localparam  use_memory = 0,
            use_ramp   = 1,
            use_adc    = 2;
reg [1:0] conf_outputSelect[3:0];
reg conf_AdcSelect[3:0];
reg conf_useLinearizer[3:0];

localparam digInpSize = $clog2(nOfDigitalPinsForTrigger-1)+1;
reg [digInpSize-1:0]conf_digitalInput[3:0];

wire [13:0] selectedADC_input[3:0];
wire selectedDigital_input[3:0];

//set output from memory
reg [pwm_size-1:0] valuesFromMemory[3:0];

//set output from ramp
localparam  trigger_none   = 0,
            trigger_now    = 1,
            trigger_adc   = 2,
            trigger_digitalPin = 3;
localparam  edge_pos    = 0,
            edge_neg    = 1;
localparam nOfRamps = 8;
localparam nOfRamps_log = $clog2(nOfRamps+1);
reg [pwm_size-1:0] ramp_start[3:0][nOfRamps-1:0];
reg [pwm_size-1:0] ramp_valueIncrementer[3:0][nOfRamps-1:0];
reg [timerSize-1:0] ramp_timeIncremeter[3:0][nOfRamps-1:0];
reg [pwm_size-1:0] ramp_nOfCycles[3:0][nOfRamps-1:0];
reg [nOfRamps_log-1:0] nOfUsedRamps[3:0];
reg useMultipleTriggers[3:0];
reg [1:0] ramp_triggerSelect[3:0];
reg [13:0] ramp_ADCtriggerValue[3:0];
reg ramp_ADCtriggerEdge[3:0];
reg [1:0] ramp_idleConfig[3:0];
wire ramp_trigger[3:0];
wire ramp_ADCtrigger[3:0];
wire [pwm_size-1:0] ramp_Output[3:0];
wire isRampBusy [3:0];

//set output from ADC
reg [scalerSize-1:0] adc_scaler[3:0];
reg [13:0] adc_minValue[3:0];
wire [pwm_size-1:0] adc_conditionedOut[3:0];


localparam nOfEdges = 8;//if you want to change it, also change the parameter .edgePoints() (and following) of the segmentedFunction linearizer

reg [pwm_size-1:0]              asg_edges     [nOfEdges-1:0];
reg [pwm_size-1:0]              asg_qs        [nOfEdges-1:0];
reg [32-pwm_size-pwm_size-1:0]  asg_ms        [nOfEdges-1:0];


reg [pwm_size-1:0] signalToLinearize[3:0];
wire [pwm_size-1:0] linearizedOut[3:0];
reg [pwm_size-1:0] outs[3:0];
integer i,j;

generate
genvar gi;

for(gi = 0; gi < 4; gi = gi + 1)begin
    
    //select input ADC
    assign selectedADC_input[gi] = conf_AdcSelect[gi] == 0 ?
                                    dat_a_i :
                                  conf_AdcSelect[gi] == 1 ?
                                    dat_b_i :
                                    0;//should never happen
                                        
    assign selectedDigital_input[gi] = digitalInputs[conf_digitalInput[gi]];
    
    //ramp
    assign ramp_ADCtrigger[gi] = ramp_ADCtriggerEdge[gi] == edge_pos ?//todo sarebbe da fare un trigger di un solo colpo di clock, oppure aggiungi alla rampa uno stato in cui aspetta che il trigger si abbassi
                                    $signed(selectedADC_input[gi]) >= $signed(ramp_ADCtriggerValue[gi]) : 
                                    $signed(selectedADC_input[gi]) <= $signed(ramp_ADCtriggerValue[gi]);
    
    assign ramp_trigger[gi] = ramp_triggerSelect[gi] == trigger_none ?
                                 0 :
                              ramp_triggerSelect[gi] == trigger_now ?
                                 1 :
                              ramp_triggerSelect[gi] == trigger_adc ?
                                 ramp_ADCtrigger[gi] :
//                            ramp_triggerSelect[gi] == trigger_digitalPin ? 
                                 selectedDigital_input[gi];
    
    //ramp setup    
    ramp#(
        .nOf_ramps(nOfRamps),
        .data_size(pwm_size),
        .time_size(timerSize)
    )rrrrrrrr(
    
        .clk            (clk_i),
        .reset          (!rstn_i),
        .trigger        (ramp_trigger[gi]),
        .defaultValue   (valuesFromMemory[gi]),
        
        .usedRamps      (nOfUsedRamps[gi]),
        .useMultipleTriggers(useMultipleTriggers[gi]),
        
//        .startPoint     (ramp_start[gi][0]),
//        .stepIncrease   (ramp_valueIncrementer[gi][0]),
//        .timeStep       (ramp_timeIncremeter[gi][0]),
//        .nOfSteps       (ramp_nOfCycles[gi][0]),

        .startPoint     ({ramp_start[gi][7], ramp_start[gi][6], ramp_start[gi][5], ramp_start[gi][4], ramp_start[gi][3], ramp_start[gi][2], ramp_start[gi][1], ramp_start[gi][0]}),
        .stepIncrease   ({ramp_valueIncrementer[gi][7], ramp_valueIncrementer[gi][6], ramp_valueIncrementer[gi][5], ramp_valueIncrementer[gi][4], ramp_valueIncrementer[gi][3], ramp_valueIncrementer[gi][2], ramp_valueIncrementer[gi][1], ramp_valueIncrementer[gi][0]}),
        .timeStep       ({ramp_timeIncremeter[gi][7], ramp_timeIncremeter[gi][6], ramp_timeIncremeter[gi][5], ramp_timeIncremeter[gi][4], ramp_timeIncremeter[gi][3], ramp_timeIncremeter[gi][2], ramp_timeIncremeter[gi][1], ramp_timeIncremeter[gi][0]}),
        .nOfSteps       ({ramp_nOfCycles[gi][7], ramp_nOfCycles[gi][6], ramp_nOfCycles[gi][5], ramp_nOfCycles[gi][4], ramp_nOfCycles[gi][3], ramp_nOfCycles[gi][2], ramp_nOfCycles[gi][1], ramp_nOfCycles[gi][0]}),
        .idleConfig     (ramp_idleConfig[gi]),
        .out            (ramp_Output[gi]),
        .isBusy         (isRampBusy[gi])
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

    segmentedFunction#(
        .nOfEdges       (nOfEdges),
        .totalBits_IO   (pwm_size),
        .fracBits_IO    (0),
        .totalBits_m    (32-pwm_size-pwm_size),
        .fracBits_m     (32-pwm_size-pwm_size-pwm_size),
        .areSignalsSigned(0)
    )linearizer(
        .clk            (clk_i),       
        .reset          (!rstn_i),     
        .in             (signalToLinearize[gi]),
        .edgePoints     ({asg_edges [7], asg_edges [6], asg_edges [5], asg_edges [4], asg_edges [3], asg_edges [2], asg_edges [1], asg_edges [0]}),
        .qs             ({asg_qs    [7], asg_qs    [6], asg_qs    [5], asg_qs    [4], asg_qs    [3], asg_qs    [2], asg_qs    [1], asg_qs    [0]}),
        .ms             ({asg_ms    [7], asg_ms    [6], asg_ms    [5], asg_ms    [4], asg_ms    [3], asg_ms    [2], asg_ms    [1], asg_ms    [0]}),
        .out            (linearizedOut[gi])
    );
              
end

endgenerate

//select output
always @(posedge clk_i) begin
    if (!rstn_i) begin
        for(i = 0; i < 4; i=i+1)begin
            signalToLinearize[i] <= 0;
            outs[i] <= 0;
        end
    end else begin
        for(i = 0; i < 4; i=i+1)begin
            signalToLinearize[i] <= conf_outputSelect[i] == use_memory ? 
                           valuesFromMemory[i] :
                       conf_outputSelect[i] == use_ramp ?
                           ramp_Output[i] :
                       conf_outputSelect[i] == use_adc ?
                           adc_conditionedOut[i] :
                           0;
            outs[i] <= conf_useLinearizer[i] ? linearizedOut[i] : signalToLinearize[i];
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
            conf_useLinearizer[i] <= 0;
            conf_digitalInput[i] <= 0;
            for(j = 0; j < nOfRamps; j=j+1)begin
                ramp_start[i][j] <= 0;
                ramp_valueIncrementer[i][j] <= 0;
                ramp_timeIncremeter[i][j] <= 0;
                ramp_nOfCycles[i][j] <= 0;
            end
            ramp_triggerSelect[i] <= 0;
            ramp_ADCtriggerValue[i] <= 0;
            ramp_ADCtriggerEdge[i] <= 0;
            nOfUsedRamps[i] <= 0;
            useMultipleTriggers[i] <= 0;
            ramp_idleConfig[i] <= 0;
            
            valuesFromMemory[i] <= 0;
            adc_minValue[i] <= 'h2000;
            adc_scaler[i] <= 1<<(scalerFracBits-1);
                        
        end
        
        for(i=0;i<nOfEdges;i=i+1)begin
            asg_edges[i] <= 0;
            asg_qs[i] <= 0;
            asg_ms[i] <= 0;
        end
    end else begin
        for(i = 0; i < 4; i=i+1)begin
            if (sys_wen) begin
                if (sys_addr[19:0] == 'h20 + (i << 2))begin//addresses 0x20, 0x24, 0x28, 0x2C
                    conf_outputSelect[i] <= sys_wdata[1:0];
                    conf_AdcSelect[i] <= sys_wdata[2];
                    conf_useLinearizer[i] <= sys_wdata[3];
                    conf_digitalInput[i] <= sys_wdata[4+digInpSize-1:4];
                    valuesFromMemory[i] <= sys_wdata[23:16];
                end
                if (sys_addr[19:0] == 'h30 + (i << 2))begin//addresses 0x30, 0x34, 0x38, 0x3C
                    adc_minValue[i] <= sys_wdata[13:0];
                    adc_scaler[i] <= sys_wdata[14+scalerSize-1:14];
                end
                
                //addresses 0x40 to 0x5C done after the for loop
                
                if (sys_addr[19:0] == 'h60 + (i << 2))begin//addresses 0x60, 0x64, 0x68, 0x6C
                    ramp_triggerSelect[i] <= sys_wdata[1:0];
                    ramp_idleConfig[i] <= sys_wdata[3:2];
                    ramp_ADCtriggerValue[i] <= sys_wdata[17:4];
                    ramp_ADCtriggerEdge[i] <= sys_wdata[18];
                    useMultipleTriggers[i] <= sys_wdata[19];
                    nOfUsedRamps[i] <= sys_wdata[31:20];//should be [20+nOfRamps_log-1:20], but I'm not gonna debug the actual number, cause I'm lazy
                end
                
                for(j = 0; j < nOfRamps; j=j+1)begin
                    if (sys_addr[19:0] == 'h70 + (j << 3) + (i * 8 * nOfRamps))begin//addresses 0x70, 0x78, 0x80 ... 
                        ramp_start[i][j] <= sys_wdata[pwm_size-1:0];
                        ramp_valueIncrementer[i][j] <= sys_wdata[pwm_size+pwm_size-1:pwm_size];
                    end
                    
                    if (sys_addr[19:0] == 'h74 + (j << 3) + (i * 8 * nOfRamps))begin//addresses 0x74, 0x7C, 0x58, 0x84
                        ramp_timeIncremeter[i][j] <= sys_wdata[timerSize-1:0];
                        ramp_nOfCycles[i][j] <= sys_wdata[timerSize+pwm_size-1:timerSize];
                    end
                end
                
                
                
            end else begin
                if(ramp_triggerSelect[i] == trigger_now)begin
                    ramp_triggerSelect[i] <= trigger_none;
                end     
            end
        end
        
        if (sys_wen) begin
            if (sys_addr[19:0] >= 'h40 && sys_addr[19:0] < 'h60)begin
                asg_edges[sys_addr[4:2]] <= sys_wdata[7:0];
                asg_qs[sys_addr[4:2]] <= sys_wdata[15:8];
                asg_ms[sys_addr[4:2]] <= sys_wdata[31:16];
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
            20'h20 : begin sys_rdata <= {valuesFromMemory[0], {(16-4-digInpSize){1'b0}},conf_digitalInput[0], conf_useLinearizer[0], conf_AdcSelect[0], conf_outputSelect[0]}; end
            20'h24 : begin sys_rdata <= {valuesFromMemory[1], {(16-4-digInpSize){1'b0}},conf_digitalInput[1], conf_useLinearizer[1], conf_AdcSelect[1], conf_outputSelect[1]}; end
            20'h28 : begin sys_rdata <= {valuesFromMemory[2], {(16-4-digInpSize){1'b0}},conf_digitalInput[2], conf_useLinearizer[2], conf_AdcSelect[2], conf_outputSelect[2]}; end
            20'h2C : begin sys_rdata <= {valuesFromMemory[3], {(16-4-digInpSize){1'b0}},conf_digitalInput[3], conf_useLinearizer[3], conf_AdcSelect[3], conf_outputSelect[3]}; end
            20'h30 : begin sys_rdata <= {adc_scaler[0], adc_minValue[0]}; end
            20'h34 : begin sys_rdata <= {adc_scaler[1], adc_minValue[1]}; end
            20'h38 : begin sys_rdata <= {adc_scaler[2], adc_minValue[2]}; end
            20'h3C : begin sys_rdata <= {adc_scaler[3], adc_minValue[3]}; end
            
            20'h40 : begin sys_rdata <= {asg_ms[0], asg_qs[0], asg_edges[0]}; end
            20'h44 : begin sys_rdata <= {asg_ms[1], asg_qs[1], asg_edges[1]}; end
            20'h48 : begin sys_rdata <= {asg_ms[2], asg_qs[2], asg_edges[2]}; end
            20'h4C : begin sys_rdata <= {asg_ms[3], asg_qs[3], asg_edges[3]}; end
            20'h50 : begin sys_rdata <= {asg_ms[4], asg_qs[4], asg_edges[4]}; end
            20'h54 : begin sys_rdata <= {asg_ms[5], asg_qs[5], asg_edges[5]}; end
            20'h58 : begin sys_rdata <= {asg_ms[6], asg_qs[6], asg_edges[6]}; end
            20'h5C : begin sys_rdata <= {asg_ms[7], asg_qs[7], asg_edges[7]}; end
            
            20'h60 : begin sys_rdata <= {nOfUsedRamps[0], useMultipleTriggers[0], ramp_ADCtriggerEdge[0], ramp_ADCtriggerValue[0], ramp_idleConfig[0], ramp_triggerSelect[0]}; end
            20'h64 : begin sys_rdata <= {nOfUsedRamps[1], useMultipleTriggers[1], ramp_ADCtriggerEdge[1], ramp_ADCtriggerValue[1], ramp_idleConfig[1], ramp_triggerSelect[1]}; end
            20'h68 : begin sys_rdata <= {nOfUsedRamps[2], useMultipleTriggers[2], ramp_ADCtriggerEdge[2], ramp_ADCtriggerValue[2], ramp_idleConfig[2], ramp_triggerSelect[2]}; end
            20'h6C : begin sys_rdata <= {nOfUsedRamps[3], useMultipleTriggers[3], ramp_ADCtriggerEdge[3], ramp_ADCtriggerValue[3], ramp_idleConfig[3], ramp_triggerSelect[3]}; end
            
            20'h070 : begin sys_rdata <= {ramp_valueIncrementer[0][0], ramp_start[0][0]}; end
            20'h074 : begin sys_rdata <= {ramp_nOfCycles[0][0], ramp_timeIncremeter[0][0]}; end
            20'h078 : begin sys_rdata <= {ramp_valueIncrementer[0][1], ramp_start[0][1]}; end
            20'h07C : begin sys_rdata <= {ramp_nOfCycles[0][1], ramp_timeIncremeter[0][1]}; end
            20'h080 : begin sys_rdata <= {ramp_valueIncrementer[0][2], ramp_start[0][2]}; end
            20'h084 : begin sys_rdata <= {ramp_nOfCycles[0][2], ramp_timeIncremeter[0][2]}; end
            20'h088 : begin sys_rdata <= {ramp_valueIncrementer[0][3], ramp_start[0][3]}; end
            20'h08C : begin sys_rdata <= {ramp_nOfCycles[0][3], ramp_timeIncremeter[0][3]}; end
            20'h090 : begin sys_rdata <= {ramp_valueIncrementer[0][4], ramp_start[0][4]}; end
            20'h094 : begin sys_rdata <= {ramp_nOfCycles[0][4], ramp_timeIncremeter[0][4]}; end
            20'h098 : begin sys_rdata <= {ramp_valueIncrementer[0][5], ramp_start[0][5]}; end
            20'h09C : begin sys_rdata <= {ramp_nOfCycles[0][5], ramp_timeIncremeter[0][5]}; end
            20'h0A0 : begin sys_rdata <= {ramp_valueIncrementer[0][6], ramp_start[0][6]}; end
            20'h0A4 : begin sys_rdata <= {ramp_nOfCycles[0][6], ramp_timeIncremeter[0][6]}; end
            20'h0A8 : begin sys_rdata <= {ramp_valueIncrementer[0][7], ramp_start[0][7]}; end
            20'h0AC : begin sys_rdata <= {ramp_nOfCycles[0][7], ramp_timeIncremeter[0][7]}; end
                        
            20'h0B0 : begin sys_rdata <= {ramp_valueIncrementer[1][0], ramp_start[1][0]}; end
            20'h0B4 : begin sys_rdata <= {ramp_nOfCycles[1][0], ramp_timeIncremeter[1][0]}; end
            20'h0B8 : begin sys_rdata <= {ramp_valueIncrementer[1][1], ramp_start[1][1]}; end
            20'h0BC : begin sys_rdata <= {ramp_nOfCycles[1][1], ramp_timeIncremeter[1][1]}; end
            20'h0C0 : begin sys_rdata <= {ramp_valueIncrementer[1][2], ramp_start[1][2]}; end
            20'h0C4 : begin sys_rdata <= {ramp_nOfCycles[1][2], ramp_timeIncremeter[1][2]}; end
            20'h0C8 : begin sys_rdata <= {ramp_valueIncrementer[1][3], ramp_start[1][3]}; end
            20'h0CC : begin sys_rdata <= {ramp_nOfCycles[1][3], ramp_timeIncremeter[1][3]}; end
            20'h0D0 : begin sys_rdata <= {ramp_valueIncrementer[1][4], ramp_start[1][4]}; end
            20'h0D4 : begin sys_rdata <= {ramp_nOfCycles[1][4], ramp_timeIncremeter[1][4]}; end
            20'h0D8 : begin sys_rdata <= {ramp_valueIncrementer[1][5], ramp_start[1][5]}; end
            20'h0DC : begin sys_rdata <= {ramp_nOfCycles[1][5], ramp_timeIncremeter[1][5]}; end
            20'h0E0 : begin sys_rdata <= {ramp_valueIncrementer[1][6], ramp_start[1][6]}; end
            20'h0E4 : begin sys_rdata <= {ramp_nOfCycles[1][6], ramp_timeIncremeter[1][6]}; end
            20'h0E8 : begin sys_rdata <= {ramp_valueIncrementer[1][7], ramp_start[1][7]}; end
            20'h0EC : begin sys_rdata <= {ramp_nOfCycles[1][7], ramp_timeIncremeter[1][7]}; end
            
            20'h0F0 : begin sys_rdata <= {ramp_valueIncrementer[2][0], ramp_start[2][0]}; end
            20'h0F4 : begin sys_rdata <= {ramp_nOfCycles[2][0], ramp_timeIncremeter[2][0]}; end
            20'h0F8 : begin sys_rdata <= {ramp_valueIncrementer[2][1], ramp_start[2][1]}; end
            20'h0FC : begin sys_rdata <= {ramp_nOfCycles[2][1], ramp_timeIncremeter[2][1]}; end
            20'h100 : begin sys_rdata <= {ramp_valueIncrementer[2][2], ramp_start[2][2]}; end
            20'h104 : begin sys_rdata <= {ramp_nOfCycles[2][2], ramp_timeIncremeter[2][2]}; end
            20'h108 : begin sys_rdata <= {ramp_valueIncrementer[2][3], ramp_start[2][3]}; end
            20'h10C : begin sys_rdata <= {ramp_nOfCycles[2][3], ramp_timeIncremeter[2][3]}; end
            20'h110 : begin sys_rdata <= {ramp_valueIncrementer[2][4], ramp_start[2][4]}; end
            20'h114 : begin sys_rdata <= {ramp_nOfCycles[2][4], ramp_timeIncremeter[2][4]}; end
            20'h118 : begin sys_rdata <= {ramp_valueIncrementer[2][5], ramp_start[2][5]}; end
            20'h11C : begin sys_rdata <= {ramp_nOfCycles[2][5], ramp_timeIncremeter[2][5]}; end
            20'h120 : begin sys_rdata <= {ramp_valueIncrementer[2][6], ramp_start[2][6]}; end
            20'h124 : begin sys_rdata <= {ramp_nOfCycles[2][6], ramp_timeIncremeter[2][6]}; end
            20'h128 : begin sys_rdata <= {ramp_valueIncrementer[2][7], ramp_start[2][7]}; end
            20'h12C : begin sys_rdata <= {ramp_nOfCycles[2][7], ramp_timeIncremeter[2][7]}; end
            
            20'h130 : begin sys_rdata <= {ramp_valueIncrementer[3][0], ramp_start[3][0]}; end
            20'h134 : begin sys_rdata <= {ramp_nOfCycles[3][0], ramp_timeIncremeter[3][0]}; end
            20'h138 : begin sys_rdata <= {ramp_valueIncrementer[3][1], ramp_start[3][1]}; end
            20'h13C : begin sys_rdata <= {ramp_nOfCycles[3][1], ramp_timeIncremeter[3][1]}; end
            20'h140 : begin sys_rdata <= {ramp_valueIncrementer[3][2], ramp_start[3][2]}; end
            20'h144 : begin sys_rdata <= {ramp_nOfCycles[3][2], ramp_timeIncremeter[3][2]}; end
            20'h148 : begin sys_rdata <= {ramp_valueIncrementer[3][3], ramp_start[3][3]}; end
            20'h14C : begin sys_rdata <= {ramp_nOfCycles[3][3], ramp_timeIncremeter[3][3]}; end
            20'h150 : begin sys_rdata <= {ramp_valueIncrementer[3][4], ramp_start[3][4]}; end
            20'h154 : begin sys_rdata <= {ramp_nOfCycles[3][4], ramp_timeIncremeter[3][4]}; end
            20'h158 : begin sys_rdata <= {ramp_valueIncrementer[3][5], ramp_start[3][5]}; end
            20'h15C : begin sys_rdata <= {ramp_nOfCycles[3][5], ramp_timeIncremeter[3][5]}; end
            20'h160 : begin sys_rdata <= {ramp_valueIncrementer[3][6], ramp_start[3][6]}; end
            20'h164 : begin sys_rdata <= {ramp_nOfCycles[3][6], ramp_timeIncremeter[3][6]}; end
            20'h168 : begin sys_rdata <= {ramp_valueIncrementer[3][7], ramp_start[3][7]}; end
            20'h16C : begin sys_rdata <= {ramp_nOfCycles[3][7], ramp_timeIncremeter[3][7]}; end
            
            
            default : begin sys_rdata <=32'h0; end
        endcase
    end
end

endmodule
