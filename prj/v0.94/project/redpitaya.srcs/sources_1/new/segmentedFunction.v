`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11.03.2024 09:12:36
// Design Name: 
// Module Name: segmentedFunction
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


module segmentedFunction#(
    parameter     nOfEdges = -1         ,
    parameter     totalBits_IO = 14     ,
    parameter     fracBits_IO = 0       ,
    parameter     totalBits_m = totalBits_IO ,
    parameter     fracBits_m = fracBits_IO,
    parameter     areSignalsSigned = 1
)(
    input                                       clk             ,
    input                                       reset           ,
    input [totalBits_IO-1:0]                    in              ,
    input [nOfEdges-1:0][totalBits_IO-1:0]      edgePoints      ,
    input [nOfEdges-1:0][totalBits_IO-1:0]      qs              ,
    input [nOfEdges-1:0][totalBits_m-1:0]       ms              ,
    output reg [totalBits_IO-1:0]               out
);

//applies to the input a segmented function. The values for edgePoints, qs and 
    //ms can be found for any segmented function as follows
    /*    
    #x and y are the sample points and values of the function ( y[i] = f(x[i]) )
    #es: x = np.linspace(0,1,8)
    #    y = np.tanh(x*4)
    a = x[0:len(x)-1]
    b = x[1:]
    c = y[0:len(y)-1]
    d = y[1:]
    edgePoints = a
    qs = c
    ms = (d-c) / (b-a)
    */
    
//if you don't use some samples, set their edgePoint to a value <= edgePoint[0] (just set them to 0 
    //or -(2^(totalBits_IO-1)), depending on if the signals are signed or not)

localparam nOfInputDelays = 3;//we'll need a bunch of delays to "synchronize" with all the operations

reg [totalBits_IO-1:0]      in_r            [nOfInputDelays-1:0];
reg [totalBits_IO-1:0]      edgePoints_r    [nOfEdges-1:0];
reg [totalBits_IO-1:0]      qs_r            [nOfEdges-1:0];
reg [totalBits_m-1:0]       ms_r            [nOfEdges-1:0];

wire [nOfEdges-1:0] isInHigherThanEdge;//bitString of the form 00...0011...11
wire [nOfEdges-1:0] isCurrentEdge;//bitString of the form      00...0010...00
reg [$clog2(nOfEdges):0] edgeIndex;

reg [totalBits_IO-1:0] current_Edge;
reg [totalBits_IO-1:0] current_q[nOfInputDelays-2:0];
reg [totalBits_m-1:0] current_m;
wire [totalBits_IO-1:0] mx;


generate
    genvar gi;
    
    //set isInHigherThanEdge    
    if(areSignalsSigned) begin
        assign isInHigherThanEdge[0] = $signed(in_r[0]) >= $signed(edgePoints_r[0]);
        for(gi = 1; gi < nOfEdges; gi = gi + 1)begin
            assign isInHigherThanEdge[gi] = ($signed(in_r[0]) >= $signed(edgePoints_r[gi])) && 
                ($signed(edgePoints_r[gi]) > $signed(edgePoints_r[gi-1]));// if set lower or higher than the first 
                                                                                    //edge, it means it is disabled
        end
    end else begin
        assign isInHigherThanEdge[0] = $unsigned(in_r[0]) >= $unsigned(edgePoints_r[0]);
        for(gi = 1; gi < nOfEdges; gi = gi + 1)begin
            assign isInHigherThanEdge[gi] = ($unsigned(in_r[0]) >= $unsigned(edgePoints_r[gi])) && 
                ($unsigned(edgePoints_r[gi]) > $unsigned(edgePoints_r[gi-1]));// if set lower or higher than the first 
                                                                                    //edge, it means it is disabled
        end    
    end
    
    //set isCurrentEdge
    assign isCurrentEdge[nOfEdges - 1] = isInHigherThanEdge[nOfEdges - 1];
    for(gi = 0; gi < nOfEdges - 1; gi = gi + 1)begin
        assign isCurrentEdge[gi] = !isInHigherThanEdge[gi + 1] & isInHigherThanEdge[gi];
    end
    
endgenerate

clocked_FractionalMultiplier #(
  .A_WIDTH			(totalBits_IO),
  .B_WIDTH			(totalBits_m),
  .OUTPUT_WIDTH		(totalBits_IO),
  .FRAC_BITS_A		(fracBits_IO),
  .FRAC_BITS_B		(fracBits_m),
  .FRAC_BITS_OUT	(fracBits_IO),
  .areSignalsSigned (areSignalsSigned)
) mult (
  .clk(clk),
  .a(in_r[nOfInputDelays-1] - current_Edge),
  .b(current_m),
  .result(mx)
);


integer i;
always @(posedge clk)begin
    if(reset)begin
        for(i=0; i < nOfInputDelays; i = i + 1)begin
            in_r[i] <= 0;
        end
        out <= 0;
        for(i=0; i < nOfEdges; i = i + 1)begin
            edgePoints_r  [i] <= 0;
            qs_r          [i] <= 0;
            ms_r          [i] <= 0;
        end
        
        edgeIndex <= 0;
        current_Edge <= 0;
        for(i=0; i < nOfInputDelays-2; i = i + 1)begin
            current_q[i] <= 0;
        end
        current_m <= 0;
        
    end else begin
        in_r[0] <= in;
        for(i=1; i < nOfInputDelays; i = i + 1)begin
            in_r[i] <= in_r[i-1];
        end
        for(i=0; i < nOfEdges; i = i + 1)begin
            edgePoints_r  [i] <= edgePoints  [i];
            qs_r          [i] <= qs          [i];
            ms_r          [i] <= ms          [i];
        end
    
        for(i=0; i < nOfEdges; i = i + 1)begin
            if(isCurrentEdge[i])begin
                edgeIndex <= i;     
            end
        end
        
        current_q[0] <= qs_r[edgeIndex];
        current_Edge <= edgePoints[edgeIndex];
        for(i=1; i < nOfInputDelays-1; i = i + 1)begin
            current_q[i] <= current_q[i-1];
        end
        
        current_m <= ms_r[edgeIndex];
        
        out <= current_q[nOfInputDelays-2] + mx;
    end
end


endmodule




















