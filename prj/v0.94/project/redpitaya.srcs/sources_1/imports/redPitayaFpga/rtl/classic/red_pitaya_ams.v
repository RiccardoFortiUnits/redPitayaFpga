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

module red_pitaya_ams (
    // ADC
    input                 clk_i           ,  // clock
    input                 rstn_i          ,  // reset - active low
    // PWM DAC
    output      [  8-1: 0] dac_a_o         ,  // values used for
    output      [  8-1: 0] dac_b_o         ,  // conversion into PWM signal
    output      [  8-1: 0] dac_c_o         ,  // 
    output      [  8-1: 0] dac_d_o         ,  // 
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

localparam  use_memory = 0,
            use_ADC0   = 2,
            use_ADC1   = 3;

parameter scalerSize = 14;
parameter scalerFracBits = 10;

//---------------------------------------------------------------------------------
//
//  System bus connection
reg [7:0] valuesFromMemory[3:0];
wire [13:0] selectedADC_input[3:0];
wire [7:0] valuesFromADC_shift_n_scaler[3:0];
reg [1:0] conf[3:0];
reg [scalerSize-1:0] scalers[3:0];
reg [13:0] minShiftValues[3:0];
reg [7:0] outs[3:0];
integer i;

//decide the outputs based on the configurations

generate
genvar gi;

for(gi = 0; gi < 4; gi = gi + 1)begin
    
    assign selectedADC_input[gi] = conf[gi] == use_ADC0 ? 
                                       dat_a_i :
                                   conf[gi] == use_ADC1 ? 
                                       dat_b_i :
                                       0;
    shift_n_scale#(
        .input_size     (14),
        .output_size    (8),
        .scaler_size    (scalerSize),
        .scaler_fracBits(scalerFracBits)
    )sns(
        .clk          (clk_i),
        .reset        (!rstn_i),
        .in           (selectedADC_input[gi]),
        .out          (valuesFromADC_shift_n_scaler[gi]),
        .minInputValue(minShiftValues[gi]),
        .scalingFactor(scalers[gi])
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
            outs[i] <= conf[i] == use_memory ? 
                           valuesFromMemory[i] :
                       conf[i] == 1 ?
                           dat_a_i[13:6] :
                           valuesFromADC_shift_n_scaler[i];
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
            valuesFromMemory[i] <= 0;
            conf[i] <= use_ADC0;
            minShiftValues[i] <= 'h2000;
            scalers[i] <= 1<<(scalerFracBits-1);
        end
    end else begin
        if (sys_wen) begin

//            if (sys_addr[19:0]==16'h20)begin
//            	valuesFromMemory[0] <= sys_wdata[23:16];
//            	conf[0] <= sys_wdata[1:0];
//            end
//            if (sys_addr[19:0]==16'h24)begin
//            	valuesFromMemory[1] <= sys_wdata[23:16];
//            	conf[1] <= sys_wdata[1:0];
//            end
//            if (sys_addr[19:0]==16'h28)begin
//            	valuesFromMemory[2] <= sys_wdata[23:16];
//            	conf[2] <= sys_wdata[1:0];
//            end
//            if (sys_addr[19:0]==16'h2C)begin
//            	valuesFromMemory[3] <= sys_wdata[23:16];
//            	conf[3] <= sys_wdata[1:0];
//            end
//            if (sys_addr[19:0]==16'h30)begin
//            	minShiftValues[0] <= sys_wdata[13:0];
//            	scalers[0] <= sys_wdata[14+scalerSize-1:14];
//            end
//            if (sys_addr[19:0]==16'h34)begin
//            	minShiftValues[1] <= sys_wdata[13:0];
//            	scalers[1] <= sys_wdata[14+scalerSize-1:14];
//            end
//            if (sys_addr[19:0]==16'h38)begin
//            	minShiftValues[2] <= sys_wdata[13:0];
//            	scalers[2] <= sys_wdata[14+scalerSize-1:14];
//            end
//            if (sys_addr[19:0]==16'h3C)begin
//            	minShiftValues[3] <= sys_wdata[13:0];
//            	scalers[3] <= sys_wdata[14+scalerSize-1:14];
//            end        
            for(i = 0; i < 4; i=i+1)begin
                if (sys_addr[19:0] == 'h20 + (i << 2))begin//addresses 0x20, 0x24, 0x28, 0x2C
                    valuesFromMemory[i] <= sys_wdata[23:16];
                    conf[i] <= sys_wdata[1:0];
                end
                if (sys_addr[19:0] == 'h30 + (i << 2))begin//addresses 0x30, 0x34, 0x38, 0x3C
                    minShiftValues[i] <= sys_wdata[13:0];
                    scalers[i] <= sys_wdata[14+scalerSize-1:14];
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
            20'h20 : begin sys_rdata <= {valuesFromMemory[0], {16-2{1'b0}}, conf[0]}; end
            20'h24 : begin sys_rdata <= {valuesFromMemory[1], {16-2{1'b0}}, conf[1]}; end
            20'h28 : begin sys_rdata <= {valuesFromMemory[2], {16-2{1'b0}}, conf[2]}; end
            20'h2C : begin sys_rdata <= {valuesFromMemory[3], {16-2{1'b0}}, conf[3]}; end
            20'h30 : begin sys_rdata <= {scalers[0], minShiftValues[0]}; end
            20'h34 : begin sys_rdata <= {scalers[1], minShiftValues[1]}; end
            20'h38 : begin sys_rdata <= {scalers[2], minShiftValues[2]}; end
            20'h3C : begin sys_rdata <= {scalers[3], minShiftValues[3]}; end
            default : begin sys_rdata <=32'h0; end
        endcase
    end
end

endmodule
