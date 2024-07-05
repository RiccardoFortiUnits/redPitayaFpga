
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
 *            |                        ?
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
   output     [ 15-1: 0] dat_a_o         ,  //!< output data CHA  //beware, it has one more bit, so the output will be between -2 and 2
  
   input[nOfDigitalPinsForTrigger-1:0] digitalInputs,
   // system bus
   input      [ 32-1: 0] sys_addr        ,  //!< bus address
   input      [ 32-1: 0] sys_wdata       ,  //!< bus write data
   input                 sys_wen         ,  //!< bus write enable
   input                 sys_ren         ,  //!< bus read enable
   output reg [ 32-1: 0] sys_rdata       ,  //!< bus read data
   output reg            sys_err         ,  //!< bus error indicator
   output reg            sys_ack         ,  //!< bus acknowledge signal
   
   output [ 4-1:0] led_o
);

localparam  PSR = 12         ;
localparam  ISR = 24         ;
localparam  DSR = 10         ;

localparam 
            led_feedback        = 0,
            led_PidEnabled = 1,
            led_linearizer            = 2,
            led_integralSaturation    = 3;
//---------------------------------------------------------------------------------
//  PID 11
localparam totalBits_coeffs = 28;
localparam fracBits_coeffs = 20;
localparam totalBits_IO = 24;
localparam wholeBits_IO = 14;
localparam fracBits_IO = totalBits_IO-wholeBits_IO;
localparam wholeBitsAfterPID_IO = 15;
localparam fracBitsAfterPID_IO = totalBits_IO-wholeBitsAfterPID_IO;

localparam nOfEdges = 8;
localparam totalBits_m = 32;
localparam fracBits_m = 24;

localparam     max_nOfCoefficients = 8;//if you want to change it, also change the parameter .coefficients() of the discreteFilter df

    reg  [ totalBits_coeffs-1: 0] sp ;
    reg  [ totalBits_coeffs-1: 0] kp ;
    reg  [ totalBits_coeffs-1: 0] ki ;
    reg  [ totalBits_coeffs-1: 0] kd ;
    reg                         irst ;
    wire [ wholeBitsAfterPID_IO-1: 0] pid_out;
    
    reg [totalBits_IO-1:0]    asg_edges   [nOfEdges-1:0];
    reg [totalBits_IO-1:0]    asg_qs      [nOfEdges-1:0];
    reg [totalBits_m-1:0]     asg_ms      [nOfEdges-1:0];
    
    reg [1:0]use_feedback				;
    reg use_fakeDelay					;
    reg [9:0] fakeDelay					;
    reg use_lpFilter					;
    reg use_linearizer					;
    reg use_genFilter					;
    reg [1:0]saturationConfiguration	;
    
    reg usePID_disableTrigger;
    reg [$clog2(nOfDigitalPinsForTrigger)-1:0] PID_disableTriggerIdx;
    
    integer i,j;
    reg  [totalBits_coeffs-1:0] coeffs[max_nOfCoefficients-1:0];
    reg [3:0] numDenSplit;
    
    localparam addr_numDenSplit = 'h60;
    localparam addr_coeffs = 'h64;
    localparam addr_linearizerCoeffs = 'hA0;
    
    wire [ totalBits_IO-1: 0] dat_pidded   ;    
    
    wire [totalBits_IO-1:0] dat_WithFeedback;
    reg [totalBits_IO-1:0] dat_genFiltered;
    reg [totalBits_IO-1:0] dat_linearized;
    
    wire [totalBits_IO-1:0] dat_genFilterOut;
    wire [totalBits_IO-1:0] dat_linearizedOut;
    
    wire PID_disableTrigger;
        
    assign pid_out = dat_linearized[fracBitsAfterPID_IO+wholeBitsAfterPID_IO-1:fracBitsAfterPID_IO];
    feedbackAdder#(totalBits_IO) fba(
        .feedbackType(use_feedback),
        .in({dat_a_i, {fracBits_IO{1'b0}}}),
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
        dat_linearized <= use_linearizer ? dat_linearizedOut : dat_pidded;
    end
    
/*
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
/*/
    assign dat_genFilterOut = dat_WithFeedback;
//* 
    wire pidSaturation, pidSaturation1, pidSaturation2;
    
    wire stopPid;
    safeSwitch sssss(
        pidSaturation | led_o[led_integralSaturation],
        clk_i,
        !rstn_i | irst,
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
      .set_sp_i     (  sp      ),  // set point
      .set_kp_i     (  kp      ),  // Kp
      .set_ki_i     (  ki      ),  // Ki
      .set_kd_i     (  kd      ),  // Kd
      .int_rst_i    (  irst    ),  // integrator reset
      .integralSaturation(integralSaturation1)
    );
    assign pidSaturation = pidSaturation1 | pidSaturation2;
    
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

assign led_o[led_feedback]  = use_feedback != 0;
assign led_o[led_PidEnabled] = ! (irst | pidReset | PID_disableTrigger);
assign led_o[led_integralSaturation] = integralSaturation1;
assign led_o[led_linearizer] = use_linearizer;


assign dat_a_o = pid_out;





//---------------------------------------------------------------------------------
//
//  System bus connection

always @(posedge clk_i) begin
    if (rstn_i == 1'b0) begin
       {saturationConfiguration, use_linearizer, use_genFilter, use_lpFilter, fakeDelay, use_fakeDelay, use_feedback} <= 0;
        sp    <= 0 ;
        kp    <= 0 ;
        ki    <= 0 ;
        kd    <= 0 ;
        irst  <=  1'b1 ;
        for(j=0;j<nOfEdges;j=j+1)begin
            asg_edges[j] <= 0;
            asg_qs[j] <= 0;
            asg_ms[j] <= 0;
        end
        PID_disableTriggerIdx <= 0;
        usePID_disableTrigger <= 0;
        numDenSplit  <= 0 ;
        for(i=0;i<max_nOfCoefficients;i=i+1)begin
          coeffs[i] <= 0;
        end
    end
   else begin
      if (sys_wen) begin
         if (sys_addr[15:0]==16'h0)    {irst} <= sys_wdata;
                      
         if (sys_addr[15:0]==16'h4)    {PID_disableTriggerIdx, usePID_disableTrigger, saturationConfiguration, use_linearizer, use_genFilter, fakeDelay, use_fakeDelay, use_feedback}  <= sys_wdata;
         if (sys_addr[15:0]==16'h10)    sp  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[15:0]==16'h14)    kp  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[15:0]==16'h18)    ki  <= sys_wdata[totalBits_coeffs-1:0] ;
         if (sys_addr[15:0]==16'h1C)    kd  <= sys_wdata[totalBits_coeffs-1:0] ;
                      
         if (sys_addr[15:0]==addr_numDenSplit)    numDenSplit  <= sys_wdata[3:0] ;
         
		for(i=0;i<max_nOfCoefficients;i=i+1)begin
			if (sys_addr[15:0]==(addr_coeffs+(i<<2)))    coeffs[i]  <= sys_wdata[totalBits_coeffs-1:0] ;
		end
		for(i=0;i<nOfEdges;i=i+1)begin
			if (sys_addr[15:0]==addr_linearizerCoeffs+(i<<3)) begin
                asg_edges[i] <= {sys_wdata[wholeBitsAfterPID_IO-1:0], {fracBitsAfterPID_IO{1'b0}}};
                asg_qs[i] <= {sys_wdata[wholeBitsAfterPID_IO*2-1:wholeBitsAfterPID_IO], {fracBitsAfterPID_IO{1'b0}}};
			end
			if (sys_addr[15:0]==addr_linearizerCoeffs+(i<<3)+4)    asg_ms[i] <= sys_wdata;
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
	casez (sys_addr[15:0])
		16'h00 : begin sys_rdata <= irst       ; end 

		16'h04 : begin  sys_rdata <= {PID_disableTriggerIdx, usePID_disableTrigger, saturationConfiguration, use_linearizer, use_genFilter, fakeDelay, use_fakeDelay, use_feedback};end
		
		16'h10 : begin sys_rdata <= sp          ; end 
		16'h14 : begin sys_rdata <= kp          ; end 
		16'h18 : begin sys_rdata <= ki          ; end 
		16'h1C : begin sys_rdata <= kd          ; end 
		
		16'h50 : begin sys_rdata <= dat_a_o                           				; end 
								
//		16'h60 : begin sys_rdata <= numDenSplit        								; end
//		16'h64 : begin sys_rdata <= coeffs[0]          								; end
//		16'h68 : begin sys_rdata <= coeffs[1]          								; end
//		16'h6C : begin sys_rdata <= coeffs[2]          								; end
//		16'h70 : begin sys_rdata <= coeffs[3]          								; end
//		16'h74 : begin sys_rdata <= coeffs[4]          								; end
//		16'h78 : begin sys_rdata <= coeffs[5]          								; end
//		16'h7C : begin sys_rdata <= coeffs[6]          								; end
//		16'h80 : begin sys_rdata <= coeffs[7]          								; end
		
        16'hA0 : begin sys_rdata <= {asg_qs[0][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[0][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hA4 : begin sys_rdata <= {asg_ms[0]}                                     ; end
        16'hA8 : begin sys_rdata <= {asg_qs[1][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[1][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hAC : begin sys_rdata <= {asg_ms[1]}                                     ; end
        16'hB0 : begin sys_rdata <= {asg_qs[2][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[2][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hB4 : begin sys_rdata <= {asg_ms[2]}                                     ; end
        16'hB8 : begin sys_rdata <= {asg_qs[3][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[3][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hBC : begin sys_rdata <= {asg_ms[3]}                                     ; end
        16'hC0 : begin sys_rdata <= {asg_qs[4][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[4][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hC4 : begin sys_rdata <= {asg_ms[4]}                                     ; end
        16'hC8 : begin sys_rdata <= {asg_qs[5][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[5][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hCC : begin sys_rdata <= {asg_ms[5]}                                     ; end
        16'hD0 : begin sys_rdata <= {asg_qs[6][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[6][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hD4 : begin sys_rdata <= {asg_ms[6]}                                     ; end
        16'hD8 : begin sys_rdata <= {asg_qs[7][totalBits_IO-1:fracBitsAfterPID_IO], asg_edges[7][totalBits_IO-1:fracBitsAfterPID_IO]}       ; end
        16'hDC : begin sys_rdata <= {asg_ms[7]}                                     ; end

    	default : begin sys_rdata <=  32'h0                              ; end
	endcase
end

endmodule
