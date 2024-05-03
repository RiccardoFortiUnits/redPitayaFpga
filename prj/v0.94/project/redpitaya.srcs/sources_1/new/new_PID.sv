`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08.01.2024 14:26:46
// Design Name: 
// Module Name: new_PID
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

/**
 * $Id: red_pitaya_pid_block.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya PID controller.
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
 * Proportional-integral-derivative (PID) controller.
 *
 *
 *        /---\         /---\      /-----------\
 *   IN --| - |----+--> | P | ---> | SUM & SAT | ---> OUT
 *        \---/    |    \---/      \-----------/
 *          ^      |                   ^  ^
 *          |      |    /---\          |  |
 *   set ----      +--> | I | ---------   |
 *   point         |    \---/             |
 *                 |                      |
 *                 |    /---\             |
 *                 ---> | D | ------------
 *                      \---/
 *
 *
 * Proportional-integral-derivative (PID) controller is made from three parts. 
 *
 * Error which is difference between set point and input signal is driven into
 * propotional, integral and derivative part. Each calculates its own value which
 * is then summed and saturated before given to output.
 *
 * Integral part has also separate input to reset integrator value to 0.
 * 
 */




module new_PID #(
   parameter     totalBits_IO = 14   ,
   parameter     fracBits_IO = 0     ,
   parameter     totalBits_coeffs = 24   ,
   parameter     fracBits_P = 12         ,
   parameter     fracBits_I = 18         ,
   parameter     fracBits_D = 10         ,
   parameter     totalBits_I_saturation = totalBits_IO,
   parameter     workingBits = 32
)
(
   // data
   input                 clk_i                         ,  // clock
   input                 rstn_i                        ,  // reset - active low
   input      [ totalBits_IO-1: 0] dat_i               ,  // input data
   output     [ totalBits_IO-1: 0] dat_o               ,  // output data

   // settings
   input      [ totalBits_IO-1: 0] set_sp_i            ,  // set point
   input      [ totalBits_coeffs-1: 0] set_kp_i        ,  // Kp
   input      [ totalBits_coeffs-1: 0] set_ki_i        ,  // Ki
   input      [ totalBits_coeffs-1: 0] set_kd_i        ,  // Kd
   input        int_rst_i                              ,  // integrator reset
   
   // warnings
   
   output       integralSaturation,
   output       outSaturation
);




//---------------------------------------------------------------------------------
//  Set point error calculation

reg  [ workingBits-1: 0] error        ;

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      error <= 0 ;
   end
   else begin
      error <= $signed(dat_i) - $signed(set_sp_i);
   end
end

//---------------------------------------------------------------------------------
//  Proportional part

reg   [workingBits-1: 0] kp_reg        ;
wire  [workingBits-1: 0] kp_mult       ;


FractionalMultiplier 
#(workingBits,totalBits_coeffs-1,workingBits,fracBits_IO,fracBits_P, fracBits_IO)fmp
(error,set_kp_i,kp_mult);

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kp_reg <= 0 ;
   end
   else begin
      kp_reg <= kp_mult;
   end
end


//---------------------------------------------------------------------------------
//  Integrator

wire   [workingBits-1: 0] ki_mult       ;
reg   [workingBits-1: 0] ki_mult_reg       ;
wire  [workingBits-1: 0] int_sum       ;
reg   [workingBits-1: 0] int_reg       ;
wire   [workingBits-1: 0] int_shr       ;

FractionalMultiplier 
#(workingBits,totalBits_coeffs-1,workingBits,fracBits_IO,fracBits_I, fracBits_IO)fmi
(error,set_ki_i,ki_mult);

saturator#(workingBits,totalBits_IO)sat_i(int_sum, int_shr, integralSaturation);
always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      int_reg  <= 0;
      ki_mult_reg <= 0;
   end
   else begin
      if (int_rst_i) begin
         int_reg <= 0; // reset
         ki_mult_reg <= 0;
      end else begin 
         int_reg <= int_shr;
         ki_mult_reg <= ki_mult;
       end
   end
end

assign int_sum = $signed(ki_mult_reg) + $signed(int_reg) ;






//---------------------------------------------------------------------------------
//  Derivative

wire  [workingBits-1: 0] kd_mult       ;
reg   [workingBits-1: 0] kd_mult_reg       ;
reg   [workingBits-1: 0] kd_reg        ;
wire  [workingBits-1: 0] kd_reg_s      ;


FractionalMultiplier 
#(workingBits,totalBits_coeffs-1,workingBits,fracBits_IO,fracBits_D, fracBits_IO)fmd
(error,set_kd_i,kd_mult);

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      kd_reg   <= 0;
   end
   else begin
      kd_reg   <= kd_mult_reg;
      kd_mult_reg  <= kd_mult;
   end
end
assign kd_reg_s = rstn_i ? 0 : $signed(kd_mult_reg) - $signed(kd_reg);













//---------------------------------------------------------------------------------
//  Sum together - saturate output

wire  [workingBits-1: 0] pid_sum     ;
wire  [workingBits-1: 0] pid_sum_saturated     ; // biggest posible bit-width
reg   [totalBits_IO-1: 0] pid_out     ;

assign pid_sum = $signed(kp_reg) + $signed(int_shr) + $signed(kd_reg_s) ;
precisionSaturator#(
    .s(workingBits), 
    .maxValue({1'b0,{(totalBits_IO-2){1'b1}},1'b0}))//=MAX_VALUE - 1 (0b0111...1110)
sat_pid(
    pid_sum, 
    pid_sum_saturated, 
    outSaturation
 );

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
      pid_out    <= 0 ;
   end
   else begin
     pid_out <= pid_sum_saturated;
   end
end







assign dat_o = pid_out[totalBits_IO-1:0];



endmodule