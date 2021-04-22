`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:57:38 04/07/2021 
// Design Name: 
// Module Name:    VGASyncPulseGenerator 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module VGASyncPulseGenerator(input clk, //50Mhz input clock
    input rst,
    output HS, VS, //Sync Pulses
    output active //High when active area is being drawn
);

localparam HS_START = 16; // hSync start (end of front porch)
localparam HS_END = 16 + 96; // hSync end (start of back porch)
localparam HA_START = 16 + 96 + 48; // end of back porch
localparam LINE = 800; // Length of entire line;

localparam VS_START = 480 + 10; // vSync start (end of front porch)
localparam VS_END = 6480 + 10 + 2; // vSync end (start of back porck)
localparam VA_START = 0;
localparam FRAME = 525; // Length of entire frame

reg [10:0] hPos = 0;
reg [10:0] vPos = 0;

reg pixelStrobe = 0;

assign HS = ~((hPos >= HS_START) & (hPos < HS_END));
assign VS = ~((vPos >= VS_START) & (vPos < VS_END));

assign active = (hPos >= HA_START) & (vPos >= VA_START);

always @(posedge clk) begin //generate pixel clock at 25MHz
    pixelStrobe = ~pixelStrobe;
end

always @(posedge clk) begin
    if (pixelStrobe) begin
        if (hPos == LINE) begin
            hPos <= 0;
            vPos <= vPos + 1;
        end else hPos <= hPos + 1;
        if (vPos == FRAME) vPos <= 0;
    end
end



endmodule