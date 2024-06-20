
//`define debug

/**
 * $Id: red_pitaya_pid.v 961 2014-01-21 11:40:39Z matej.oblak $
 *
 * @brief Red Pitaya MIMO PID controller.
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
 * Multiple input multiple output controller.
 *
 *
 *                 /-------\       /-----------\
 *   CHA -----+--> | PID11 | ------| SUM & SAT | ---> CHA
 *            |    \-------/       \-----------/
 *            |                            ^
 *            |    /-------\               |
 *            ---> | PID21 | ----------    |
 *                 \-------/           |   |
 *                                     |   |
 *  INPUT                              |   |         OUTPUT
 *                                     |   |
 *                 /-------\           |   |
 *            ---> | PID12 | --------------
 *            |    \-------/           |    
 *            |                        ˇ
 *            |    /-------\       /-----------\
 *   CHB -----+--> | PID22 | ------| SUM & SAT | ---> CHB
 *                 \-------/       \-----------/
 *
 *
 * MIMO controller is build from four equal submodules, each can have 
 * different settings.
 *
 * Each output is sum of two controllers with different input. That sum is also
 * saturated to protect from wrapping.
 * 
 */



module red_pitaya_pid #(
    parameter nOfDigitalPinsForTrigger = 16
)(
   // signals
   input                 clk_i           ,  //!< processing clock
   input                 rstn_i          ,  //!< processing reset - active low
   input      [ 14-1: 0] dat_a_i         ,  //!< input data CHA
   input      [ 14-1: 0] dat_b_i         ,  //!< input data CHB
   output     [ 15-1: 0] dat_a_o         ,  //!< output data CHA  //beware, it has one more bit, so the output will be between -2 and 2
   output     [ 15-1: 0] dat_b_o         ,  //!< output data CHB
  
   input[nOfDigitalPinsForTrigger-1:0] digitalInputs,
   // system bus
   input      [ 32-1: 0] sys_addr        ,  //!< bus address
   input      [ 32-1: 0] sys_wdata       ,  //!< bus write data
   input                 sys_wen         ,  //!< bus write enable
   input                 sys_ren         ,  //!< bus read enable
   output reg [ 32-1: 0] sys_rdata       ,  //!< bus read data
   output reg            sys_err         ,  //!< bus error indicator
   output reg            sys_ack         ,  //!< bus acknowledge signal
   
   output [ 8-1:0] led_o
);

localparam  PSR = 12         ;
localparam  ISR = 24         ;
localparam  DSR = 10         ;

localparam led_feedbackType0         = 0,
          led_feedbackType1         = 1,
          led_delay                 = 2,
          led_filter                = 3,
          led_PID                   = 4,
          led_linearizer            = 5,
          led_outSaturation         = 6,
          led_integralSaturation    = 7;

//---------------------------------------------------------------------------------
//  PID 11
localparam totalBits_coeffs = 28;
localparam fracBits_coeffs = 20;
localparam totalBits_IO = 24;
localparam wholeBits_IO = 14;
localparam fracBits_IO = totalBits_IO-wholeBits_IO;
localparam wholeBitsAfterPID_IO = 15;
localparam fracBitsAfterPID_IO = totalBits_IO-wholeBitsAfterPID_IO;
wire [ totalBits_IO-1: 0] dat_pidded   ;
wire [ totalBits_IO-1: 0] dat_pidded_opposite;
wire [ wholeBitsAfterPID_IO-1: 0] pid_11_out   ;
reg  [ totalBits_coeffs-1: 0] set_11_sp    ;
reg  [ totalBits_coeffs-1: 0] set_11_kp    ;
reg  [ totalBits_coeffs-1: 0] set_11_ki    ;
reg  [ totalBits_coeffs-1: 0] set_11_kd    ;
reg             set_11_irst  ;

wire [ wholeBitsAfterPID_IO-1: 0] pid_12_out   ;
reg  [ 14-1: 0] set_12_sp    ;
reg  [ 14-1: 0] set_12_kp    ;
reg  [ 20-1: 0] set_12_ki    ;
reg  [ 14-1: 0] set_12_kd    ;
reg             set_12_irst  ;


reg [1:0]use_feedback;
reg use_fakeDelay;
reg [9:0] fakeDelay;
reg use_lpFilter;
reg [31+16:0] filterCoefficient;
reg use_linearizer;
reg use_genFilter;
reg [1:0]saturationConfiguration;
reg removeCommonMode;

localparam nOfEdges = 8;
localparam totalBits_m = 32;
localparam fracBits_m = 24;

localparam     max_nOfCoefficients = 8;//if you want to change it, also change the parameter .coefficients() of the discreteFilter df

reg [totalBits_IO-1:0]    asg_edges   [nOfEdges-1:0];
reg [totalBits_IO-1:0]    asg_qs      [nOfEdges-1:0];
reg [totalBits_m-1:0]     asg_ms      [nOfEdges-1:0];

wire [wholeBits_IO-1:0] dat_commonModeRemoved;
wire [totalBits_IO-1:0] dat_WithFeedback;
reg [totalBits_IO-1:0] dat_genFiltered;
reg [totalBits_IO-1:0] dat_genFiltered_opposite;
reg [totalBits_IO-1:0] dat_linearized;
reg [totalBits_IO-1:0] dat_linearized_opposite;

wire [totalBits_IO-1:0] dat_genFilterOut;
wire [totalBits_IO-1:0] dat_genFilterOut_opposite;
wire [totalBits_IO-1:0] dat_linearizedOut;

reg usePID_disableTrigger;
reg [$clog2(nOfDigitalPinsForTrigger)-1:0] PID_disableTriggerIdx;
wire PID_disableTrigger;

assign dat_commonModeRemoved = removeCommonMode ? dat_a_i - dat_b_i : dat_a_i;

assign pid_11_out = dat_linearized[fracBitsAfterPID_IO+wholeBitsAfterPID_IO-1:fracBitsAfterPID_IO];
assign pid_12_out = dat_linearized_opposite[fracBitsAfterPID_IO+wholeBitsAfterPID_IO-1:fracBitsAfterPID_IO];
feedbackAdder#(totalBits_IO) fba(
    .feedbackType(use_feedback),
    .in({dat_commonModeRemoved, {fracBits_IO{1'b0}}}),
    .feedback(dat_pidded),//warning: we should shift this back to have the normal amount of whole bits, but it 
                            //would require a saturator. Since this module will be deprecated soon, I'm not going 
                            //to modify it. It still works, but it would be like the feedback is divided by 2
    .out(dat_WithFeedback)
    );
    
triggerCleaner_hold_n_release tc_hnr(
    .clk		(clk_i),
    .reset		(!rstn_i),
    .in			(usePID_disableTrigger & digitalInputs[PID_disableTriggerIdx]),
    .out		(PID_disableTrigger)
);

always @(*)begin
    dat_genFiltered <= use_genFilter ? dat_genFilterOut : dat_WithFeedback;
    dat_genFiltered_opposite <= use_genFilter ? dat_genFilterOut_opposite : dat_WithFeedback;
    dat_linearized <= use_linearizer ? dat_linearizedOut : dat_pidded;
    dat_linearized_opposite <= dat_pidded_opposite;
end

wire pidSaturation, pidSaturation1, pidSaturation2;

wire stopPid;
blinker bl(
    clk_i,
    stopPid,
    !rstn_i || set_11_irst,
    led_o[led_outSaturation]
);
safeSwitch sssss(
    pidSaturation | led_o[led_integralSaturation],
    clk_i,
    !rstn_i | set_11_irst,
    saturationConfiguration,
    1,
    stopPid
);

wire pidReset;
assign pidReset = rstn_i & !stopPid;

wire integralSaturation1, integralSaturation2;
new_PID #(
   .totalBits_IO               (totalBits_IO),
   .fracBits_IO                (fracBitsAfterPID_IO),
   .totalBits_coeffs           (totalBits_coeffs),
   .fracBits_P                 (PSR),
   .fracBits_I                 (ISR),
   .fracBits_D                 (DSR),
   .totalBits_I_saturation     (24),
   .workingBits                (32)
) i_pid11 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  pidReset         ),  // reset - active low
  .stopUpdate   (  PID_disableTrigger),
  .dat_i        (  {dat_genFiltered[totalBits_IO-1], dat_genFiltered[totalBits_IO-1:1]}),  // input data, 
                                         //shifted by one, so that it has the new amount of whole bits for the PID
  .dat_o        (  dat_pidded     ),  // output data

   // settings
  .set_sp_i     (  set_11_sp      ),  // set point
  .set_kp_i     (  set_11_kp      ),  // Kp
  .set_ki_i     (  set_11_ki      ),  // Ki
  .set_kd_i     (  set_11_kd      ),  // Kd
  .int_rst_i    (  set_11_irst    ),  // integrator reset
  .outSaturation(pidSaturation1),
  .integralSaturation(integralSaturation1)
);
new_PID #(
   .totalBits_IO               (totalBits_IO),
   .fracBits_IO                (fracBits_IO),
   .totalBits_coeffs           (totalBits_coeffs),
   .fracBits_P                 (PSR),
   .fracBits_I                 (ISR),
   .fracBits_D                 (DSR),
   .totalBits_I_saturation     (24),
   .workingBits                (32)
) i_pid12 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  pidReset         ),  // reset - active low
  .stopUpdate   (  stopUpdate),
  .dat_i        (  dat_genFiltered_opposite   ),  // input data
  .dat_o        (  dat_pidded_opposite     ),  // output data

   // settings
  .set_sp_i     (  set_12_sp      ),  // set point
  .set_kp_i     (  set_12_kp      ),  // Kp
  .set_ki_i     (  set_12_ki      ),  // Ki
  .set_kd_i     (  set_12_kd      ),  // Kd
  .int_rst_i    (  set_12_irst    ),  // integrator reset
  .outSaturation(pidSaturation2),
  .integralSaturation(integralSaturation2)
);
assign pidSaturation = pidSaturation1 | pidSaturation2;
assign led_o[led_integralSaturation] = integralSaturation1 | integralSaturation2;

segmentedFunction#(
    .nOfEdges		(nOfEdges),
    .totalBits_IO	(totalBits_IO),
    .fracBits_IO	(fracBitsAfterPID_IO),
    .totalBits_m	(totalBits_m),
    .fracBits_m		(fracBits_m),
    .areSignalsSigned(1)
)linearizer1(
    .clk            (clk_i),       
    .reset          (!rstn_i),     
    .in             (dat_pidded),
    .edgePoints     ({asg_edges [7], asg_edges [6], asg_edges [5], asg_edges [4], asg_edges [3], asg_edges [2], asg_edges [1], asg_edges [0]}),
    .qs             ({asg_qs    [7], asg_qs    [6], asg_qs    [5], asg_qs    [4], asg_qs    [3], asg_qs    [2], asg_qs    [1], asg_qs    [0]}),
    .ms             ({asg_ms    [7], asg_ms    [6], asg_ms    [5], asg_ms    [4], asg_ms    [3], asg_ms    [2], asg_ms    [1], asg_ms    [0]}),
    .out            (dat_linearizedOut)
);

assign led_o[led_feedbackType1:led_feedbackType0]  = use_feedback;
assign led_o[led_delay]                            = use_fakeDelay;
assign led_o[led_filter]                           = use_lpFilter;
assign led_o[led_PID]                              = rstn_i;
assign led_o[led_linearizer]                   = use_linearizer;

//---------------------------------------------------------------------------------
//  PID 21

wire [ 14-1: 0] pid_21_out   ;
reg  [ 14-1: 0] set_21_sp    ;
reg  [ 14-1: 0] set_21_kp    ;
reg  [ 20-1: 0] set_21_ki    ;
reg  [ 14-1: 0] set_21_kd    ;
reg             set_21_irst  ;

red_pitaya_pid_block #(
  .PSR (  PSR   ),
  .ISR (  ISR   ),
  .DSR (  DSR   )      
) i_pid21 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_i        (  dat_a_i        ),  // input data
  .dat_o        (  pid_21_out     ),  // output data

   // settings
  .set_sp_i     (  set_21_sp      ),  // set point
  .set_kp_i     (  set_21_kp      ),  // Kp
  .set_ki_i     (  set_21_ki      ),  // Ki
  .set_kd_i     (  set_21_kd      ),  // Kd
  .int_rst_i    (  set_21_irst    )   // integrator reset
);

//---------------------------------------------------------------------------------
//  PID 12

//red_pitaya_pid_block #(
//  .PSR (  PSR   ),
//  .ISR (  ISR   ),
//  .DSR (  DSR   )      
//) i_pid12 (
//   // data
//  .clk_i        (  clk_i          ),  // clock
//  .rstn_i       (  rstn_i         ),  // reset - active low
//  .dat_i        (  dat_b_i        ),  // input data
//  .dat_o        (  pid_12_out     ),  // output data

//   // settings
//  .set_sp_i     (  set_12_sp      ),  // set point
//  .set_kp_i     (  set_12_kp      ),  // Kp
//  .set_ki_i     (  set_12_ki      ),  // Ki
//  .set_kd_i     (  set_12_kd      ),  // Kd
//  .int_rst_i    (  set_12_irst    )   // integrator reset
//);

//---------------------------------------------------------------------------------
//  PID 22

wire [ 14-1: 0] pid_22_out   ;
reg  [ 14-1: 0] set_22_sp    ;
reg  [ 14-1: 0] set_22_kp    ;
reg  [ 20-1: 0] set_22_ki    ;
reg  [ 14-1: 0] set_22_kd    ;
reg             set_22_irst  ;

red_pitaya_pid_block #(
  .PSR (  PSR   ),
  .ISR (  ISR   ),
  .DSR (  DSR   )      
) i_pid22 (
   // data
  .clk_i        (  clk_i          ),  // clock
  .rstn_i       (  rstn_i         ),  // reset - active low
  .dat_i        (  dat_b_i        ),  // input data
  .dat_o        (  pid_22_out     ),  // output data

   // settings
  .set_sp_i     (  set_22_sp      ),  // set point
  .set_kp_i     (  set_22_kp      ),  // Kp
  .set_ki_i     (  set_22_ki      ),  // Ki
  .set_kd_i     (  set_22_kd      ),  // Kd
  .int_rst_i    (  set_22_irst    )   // integrator reset
);

//---------------------------------------------------------------------------------
//  Sum and saturation

//wire [ 15-1: 0] out_1_sum   ;
//reg  [ 14-1: 0] out_1_sat   ;
//wire [ 15-1: 0] out_2_sum   ;
//reg  [ 14-1: 0] out_2_sat   ;

//assign out_1_sum = $signed(pid_11_out);
//assign out_2_sum = $signed(pid_12_out);

//always @(posedge clk_i) begin
//   if (rstn_i == 1'b0) begin
//      out_1_sat <= 14'd0 ;
//      out_2_sat <= 14'd0 ;
//   end
//   else begin
//      if (out_1_sum[15-1:15-2]==2'b01) // postitive sat
//         out_1_sat <= 14'h1FFF ;
//      else if (out_1_sum[15-1:15-2]==2'b10) // negative sat
//         out_1_sat <= 14'h2000 ;
//      else
//         out_1_sat <= out_1_sum[14-1:0] ;

//      if (out_2_sum[15-1:15-2]==2'b01) // postitive sat
//         out_2_sat <= 14'h1FFF ;
//      else if (out_2_sum[15-1:15-2]==2'b10) // negative sat
//         out_2_sat <= 14'h2000 ;
//      else
//         out_2_sat <= out_2_sum[14-1:0] ;
//   end
//end

assign dat_a_o = pid_11_out ;
assign dat_b_o = pid_12_out ;

//---------------------------------------------------------------------------------
//
//  generic filter

integer i;
reg  [totalBits_coeffs-1:0] coeffs[max_nOfCoefficients-1:0];
reg [7:0] numDenSplit;

localparam addr_numDenSplit = 'h60;
localparam addr_coeffs = 'h64;
localparam addr_linearizerCoeffs = 'hA0;

discreteFilter #(
   .totalBits_IO               (totalBits_IO    ),
   .fracBits_IO                (fracBits_IO     ),
   .totalBits_coeffs           (totalBits_coeffs),
   .fracBits_coeffs            (fracBits_coeffs) 
) df (
   // data
  .clk        (  clk_i          ),
  .reset       (  !rstn_i         ),
  .in        (  dat_WithFeedback   ),
  .out        (  dat_genFilterOut     ),

   // settings
  .coefficients     ({coeffs[7],coeffs[6],coeffs[5],coeffs[4],coeffs[3],coeffs[2],coeffs[1],coeffs[0]}),
  .denNumSplit     (  numDenSplit      )
);

reg [totalBits_IO-1:0] dat_WithFeedback_delayed[2:0];
assign dat_genFilterOut_opposite = dat_WithFeedback_delayed[2] - dat_genFilterOut;
always @(posedge clk_i)begin
    dat_WithFeedback_delayed[0] <= dat_WithFeedback;
    dat_WithFeedback_delayed[1] <= dat_WithFeedback_delayed[0];
    dat_WithFeedback_delayed[2] <= dat_WithFeedback_delayed[1];
end


//---------------------------------------------------------------------------------
//
//  System bus connection

always @(posedge clk_i) begin
   if (rstn_i == 1'b0) begin
`ifndef debug
      {saturationConfiguration, use_linearizer, use_genFilter, use_lpFilter, fakeDelay, use_fakeDelay, use_feedback} <= 0;
      filterCoefficient <= 0;
      set_11_sp    <= 0 ;
      set_11_kp    <= 0 ;
      set_11_ki    <= 0 ;
      set_11_kd    <= 0 ;
      set_11_irst  <=  1'b1 ;
      set_12_sp    <= 14'd0 ;
      set_12_kp    <= 14'd0 ;
      set_12_ki    <= 20'd0 ;
      set_12_kd    <= 14'd0 ;
      set_12_irst  <=  1'b1 ;
      set_21_sp    <= 14'd0 ;
      set_21_kp    <= 14'd0 ;
      set_21_ki    <= 20'd0 ;
      set_21_kd    <= 14'd0 ;
      set_21_irst  <=  1'b1 ;
      set_22_sp    <= 14'd0 ;
      set_22_kp    <= 14'd0 ;
      set_22_ki    <= 20'd0 ;
      set_22_kd    <= 14'd0 ;
      set_22_irst  <=  1'b1 ;
      numDenSplit  <= 0 ;
      for(i=0;i<max_nOfCoefficients;i=i+1)begin
          coeffs[i] <= 0;
      end
      for(i=0;i<nOfEdges;i=i+1)begin
            asg_edges[i] <= 0;
            asg_qs[i] <= 0;
            asg_ms[i] <= 0;
      end
      PID_disableTriggerIdx <= 0;
      usePID_disableTrigger <= 0;
`else
      saturationConfiguration <= 0;
      use_linearizer <= 0;
      use_genFilter <= 0;
      use_lpFilter <= 0; 
      fakeDelay <= 0;
      use_fakeDelay <= 0;
      use_feedback <= 0;
      filterCoefficient <= 0;
      set_11_sp    <= 0;
      set_11_kp    <= 1 << PSR ;//1     
      set_11_ki    <= 1 << (ISR-1);//0.5
      set_11_kd    <= 0 ;
      set_11_irst  <=  1'b0 ;//Integral enabled
      set_12_sp    <= 14'd0 ;
      set_12_kp    <= 14'd0 ;
      set_12_ki    <= 20'd0 ;
      set_12_kd    <= 14'd0 ;
      set_12_irst  <=  1'b1 ;
      set_21_sp    <= 14'd0 ;
      set_21_kp    <= 14'd0 ;
      set_21_ki    <= 20'd0 ;
      set_21_kd    <= 14'd0 ;
      set_21_irst  <=  1'b1 ;
      set_22_sp    <= 14'd0 ;
      set_22_kp    <= 14'd0 ;
      set_22_ki    <= 20'd0 ;
      set_22_kd    <= 14'd0 ;
      set_22_irst  <=  1'b1 ;
      numDenSplit  <= 0 ;
      for(i=0;i<max_nOfCoefficients;i=i+1)begin
          coeffs[i] <= 0;
      end
        asg_edges[0] <= 1<<totalBits_IO;//-1
        asg_qs[0] <= -(1<<(totalBits_IO-2));//-0.25
        asg_ms[0] <= 1<<(fracBits_m-1);//0.5
        asg_edges[1] <= 0;
        asg_qs[1] <= 0;
        asg_ms[1] <= -(1<<(fracBits_m-1));//-0.5;
      for(i=2;i<nOfEdges;i=i+1)begin
            asg_edges[i] <= 0;
            asg_qs[i] <= 0;
            asg_ms[i] <= 0;
      end
      PID_disableTriggerIdx <= 0;
      usePID_disableTrigger <= 0;
`endif
   end
   else begin
      if (sys_wen) begin
         if (sys_addr[19:0]==16'h0)    {set_22_irst,set_21_irst,set_12_irst,set_11_irst} <= sys_wdata[ 4-1:0] ;
         
         if (sys_addr[19:0]==16'h4)    {removeCommonMode, saturationConfiguration, use_linearizer, use_genFilter, use_lpFilter, fakeDelay, use_fakeDelay, use_feedback}  <= sys_wdata[1+2+1+10+1+1+1+2-1:0] ;
         if (sys_addr[19:0]==16'h8)     filterCoefficient  <= {{16{sys_wdata[32-1]}},sys_wdata[30-1:0]};
         if (sys_addr[19:0]==16'hC)     {PID_disableTriggerIdx, usePID_disableTrigger}  <= sys_wdata;
         
         if (sys_addr[19:0]==16'h10)    set_11_sp  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[19:0]==16'h14)    set_11_kp  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[19:0]==16'h18)    set_11_ki  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[19:0]==16'h1C)    set_11_kd  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[19:0]==16'h20)    set_12_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h24)    set_12_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h28)    set_12_ki  <= sys_wdata[20-1:0] ;
         if (sys_addr[19:0]==16'h2C)    set_12_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h30)    set_21_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h34)    set_21_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h38)    set_21_ki  <= sys_wdata[20-1:0] ;
         if (sys_addr[19:0]==16'h3C)    set_21_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h40)    set_22_sp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h44)    set_22_kp  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==16'h48)    set_22_ki  <= sys_wdata[20-1:0] ;
         if (sys_addr[19:0]==16'h4C)    set_22_kd  <= sys_wdata[14-1:0] ;
         if (sys_addr[19:0]==addr_numDenSplit)    numDenSplit  <= sys_wdata[7:0] ;
         
		for(i=0;i<max_nOfCoefficients;i=i+1)begin
			if (sys_addr[19:0]==(addr_coeffs+(i<<2)))    coeffs[i]  <= sys_wdata[totalBits_coeffs-1:0] ;
		end
		for(i=0;i<nOfEdges;i=i+1)begin
			if (sys_addr[19:0]==addr_linearizerCoeffs+(i<<3)) begin
                asg_edges[i] <= {sys_wdata[wholeBitsAfterPID_IO-1:0], {fracBitsAfterPID_IO{1'b0}}};
                asg_qs[i] <= {sys_wdata[wholeBitsAfterPID_IO*2-1:wholeBitsAfterPID_IO], {fracBitsAfterPID_IO{1'b0}}};
			end
			if (sys_addr[19:0]==addr_linearizerCoeffs+(i<<3)+4)    asg_ms[i] <= sys_wdata;
		end
      end
   end
end

wire sys_en;
assign sys_en = sys_wen | sys_ren;

always @(posedge clk_i)
if (rstn_i == 1'b0) begin
   sys_err <= 1'b0 ;
   sys_ack <= 1'b0 ;
end else begin
   sys_err <= 1'b0 ;
    sys_ack <= sys_en;
	casez (sys_addr[19:0])
		20'h00 : begin sys_rdata <= {{32- 4{1'b0}}, set_22_irst,set_21_irst,set_12_irst,set_11_irst}       ; end 

		20'h04 : begin  sys_rdata <= {{(32-(1+2+1+10+1+1+1+2)){1'b0}}, removeCommonMode, saturationConfiguration, use_linearizer, use_genFilter, use_lpFilter, fakeDelay, use_fakeDelay, use_feedback};end
		20'h08 : begin  sys_rdata <= {{32-30{1'b0}}, filterCoefficient};end
        20'h0C : begin  sys_rdata <= {PID_disableTriggerIdx, usePID_disableTrigger}; end

		20'h10 : begin sys_rdata <= set_11_sp          ; end 
		20'h14 : begin sys_rdata <= set_11_kp          ; end 
		20'h18 : begin sys_rdata <= set_11_ki          ; end 
		20'h1C : begin sys_rdata <= set_11_kd          ; end 

		20'h20 : begin sys_rdata <= {{32-14{1'b0}}, set_12_sp}        				; end 
		20'h24 : begin sys_rdata <= {{32-14{1'b0}}, set_12_kp}        				; end 
		20'h28 : begin sys_rdata <= {{32-20{1'b0}}, set_12_ki}        				; end 
		20'h2C : begin sys_rdata <= {{32-14{1'b0}}, set_12_kd}        				; end 
								
		20'h30 : begin sys_rdata <= {{32-14{1'b0}}, set_21_sp}        				; end 
		20'h34 : begin sys_rdata <= {{32-14{1'b0}}, set_21_kp}        				; end 
		20'h38 : begin sys_rdata <= {{32-20{1'b0}}, set_21_ki}        				; end 
		20'h3C : begin sys_rdata <= {{32-14{1'b0}}, set_21_kd}        				; end 
								
		20'h40 : begin sys_rdata <= {{32-14{1'b0}}, set_22_sp}        				; end 
		20'h44 : begin sys_rdata <= {{32-14{1'b0}}, set_22_kp}        				; end 
		20'h48 : begin sys_rdata <= {{32-20{1'b0}}, set_22_ki}        				; end 
		20'h4C : begin sys_rdata <= {{32-14{1'b0}}, set_22_kd}        				; end 
		
		20'h50 : begin sys_rdata <= dat_a_o                           				; end 
		20'h54 : begin sys_rdata <= dat_b_o                           				; end 
								
		20'h60 : begin sys_rdata <= numDenSplit        								; end
		20'h64 : begin sys_rdata <= coeffs[0]          								; end
		20'h68 : begin sys_rdata <= coeffs[1]          								; end
		20'h6C : begin sys_rdata <= coeffs[2]          								; end
		20'h70 : begin sys_rdata <= coeffs[3]          								; end
		20'h74 : begin sys_rdata <= coeffs[4]          								; end
		20'h78 : begin sys_rdata <= coeffs[5]          								; end
		20'h7C : begin sys_rdata <= coeffs[6]          								; end
		20'h80 : begin sys_rdata <= coeffs[7]          								; end

        20'hA0 : begin sys_rdata <= {asg_qs[0][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[0][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hA4 : begin sys_rdata <= {asg_ms[0]}                                     ; end
        20'hA8 : begin sys_rdata <= {asg_qs[1][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[1][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hAC : begin sys_rdata <= {asg_ms[1]}                                     ; end
        20'hB0 : begin sys_rdata <= {asg_qs[2][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[2][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hB4 : begin sys_rdata <= {asg_ms[2]}                                     ; end
        20'hB8 : begin sys_rdata <= {asg_qs[3][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[3][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hBC : begin sys_rdata <= {asg_ms[3]}                                     ; end
        20'hC0 : begin sys_rdata <= {asg_qs[4][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[4][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hC4 : begin sys_rdata <= {asg_ms[4]}                                     ; end
        20'hC8 : begin sys_rdata <= {asg_qs[5][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[5][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hCC : begin sys_rdata <= {asg_ms[5]}                                     ; end
        20'hD0 : begin sys_rdata <= {asg_qs[6][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[6][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hD4 : begin sys_rdata <= {asg_ms[6]}                                     ; end
        20'hD8 : begin sys_rdata <= {asg_qs[7][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[7][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        20'hDC : begin sys_rdata <= {asg_ms[7]}                                     ; end

    	default : begin sys_rdata <=  32'h0                              ; end
	endcase
end

endmodule
