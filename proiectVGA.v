`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:24:35 04/07/2021 
// Design Name: 
// Module Name:    proiectVGA 
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

module proiectVGA(
  input clock,
  input wire right,
  input wire left,
  input wire shot,
  input wire resetValues,
  output reg [2:0 ] red, 
  output reg [2:0 ] green, 
  output reg [1:0 ] blue,
  output reg hsync, 
  output reg vsync,
  output reg [7:0] SSEG_CA,
  output reg [7:0] SSEG_AN,
  output reg [7:0] LEDS,
  output speaker
);

reg [9:0] hcount = 0;
reg [9:0] vcount = 0;
reg [1:0] counter = 0;
reg [30:0] delay = 0;
reg enable;
integer counter_time;
integer right_counter = 0;
integer fire = 0;
integer munition_position = 0;
integer munition_position_orizont = 0;
integer faier_counter = 0;
integer g;
integer permision = 1;	
integer add = 0;
integer number = 0;
integer enemyAltitude=0;
integer killCounter=0;



//------------------------------------------------------------



parameter h_pulse   = 96;	//H-SYNC pulse width 96 * 40 ns (25 Mhz) = 3.84 uS
parameter h_bp      = 48;	//H-BP back porch pulse width
parameter h_pixels  = 640;	//H-PIX Number of pixels horisontally
parameter h_fp      = 16;	//H-FP front porch pulse width
parameter h_pol     = 1'b0;	//H-SYNC polarity
parameter h_frame   = 800;	//800 = 96 (H-SYNC) + 48 (H-BP) + 640 (H-PIX) + 16 (H-FP)
parameter v_pulse   = 2;	//V-SYNC pulse width
parameter v_bp      = 33;	//V-BP back porch pulse width
parameter v_pixels  = 480;	//V-PIX Number of pixels vertically
parameter v_fp      = 10;	//V-FP front porch pulse width
parameter v_pol     = 1'b1;	//V-SYNC polarity
parameter v_frame   = 525;	// 525 = 2 (V-SYNC) + 33 (V-BP) + 480 (V-PIX) + 10 (V-FP)

parameter square_size = 10;	//size of the square we will move
parameter init_x = 320;		//initial square position X
parameter init_y = 240;		//initial square position Y

reg	[1:0]		clk_div;	// 2 bit counter
wire			vga_clk;	

assign 	vga_clk 	= clk_div[1];		// 25Mhz clock = 100Mhz divided by 2-bit counter

always @ (posedge clock) begin		// 2-bt counter ++ on each positive edge of 100Mhz clock
	clk_div <= clk_div + 2'b1;
end

//reg     [2:0]   	vga_r_r;	//VGA color registers R,G,B x 3 bit
//reg     [2:0]   	vga_g_r;
//reg     [2:0]   	vga_b_r;
//reg             	vga_hs_r;	//H-SYNC register
//reg             	vga_vs_r;	//V-SYNC register

//assign 	red 		= vga_r_r;		//assign the output signals for VGA to the VGA registers
//assign 	green 	= vga_g_r;
//assign 	blue 		= vga_b_r;
//assign 	hsync 	= vga_hs_r;
//assign 	vsync 	= vga_vs_r;

reg     [7:0]		timer_t = 8'b0;	// 8 bit timer with 0 initialization
reg             	reset = 1;	
reg     [9:0]   	c_row;		//complete frame register row
reg     [9:0]   	c_col;		//complete frame register colum
reg     [9:0]   	c_hor;		//visible frame register horisontally
reg     [9:0]   	c_ver;		//visible  frame register vertically

reg			disp_en;	//display enable flag

reg	[9:0]		sq_pos_x;	//position of square center X, Y
reg	[9:0]		sq_pos_y;

wire	[9:0]		l_sq_pos_x;	//upper left and down right corners of the square
wire	[9:0]		r_sq_pos_x;
wire	[9:0]		u_sq_pos_y;
wire	[9:0]		d_sq_pos_y;

assign 	l_sq_pos_x 	= sq_pos_x - square_size;
assign 	r_sq_pos_x 	= sq_pos_x + square_size;
assign 	u_sq_pos_y 	= sq_pos_y - square_size;
assign 	d_sq_pos_y 	= sq_pos_y + square_size;

//reg	[3:0]		ps2_cntr;		// 4-bit PS2 clock counter
//reg	[7:0]		ps2_data_reg;		// 8-bit PS2 data register
//reg	[7:0]		ps2_data_reg_prev;	// previous 8-bit PS data register
//reg	[7:0]		ps2_data_reg_prev1;	// previous previous 8-bit data register
//reg	[10:0]		ps2_dat_r;		// 11-bit complete PS2 frame register

//reg	[1:0]		ps2_clk_buf;		// PS2 clock buffer
//wire			ps2_clk_pos;		// PS2 positive edge detected signal

reg			u_arr = 0;		//PS2 arrow keys detect flags
reg			l_arr = 0;
reg			d_arr = 0;
reg			r_arr = 0;

reg	[20:0]	arr_timer;	// delay between key service

reg	[19:0]	sq_figure	[0:19];

wire	[4:0]	sq_fig_x;
wire	[4:0]	sq_fig_y;

assign sq_fig_x = c_col - l_sq_pos_x;			// our figure's x axis when in square boundary
assign sq_fig_y = c_row - u_sq_pos_y;			// our figure's y axis when in square boundary



//--------------------------------------------------------
always @ (posedge clock)
begin
  if (counter == 3)
  begin
    counter <= 1'b0;
    enable <= 1'b1;
  end
  else
  begin
    counter <= counter + 1'b1;
    enable <= 1'b0;
  end
end




  slow_clock S1 (clock, Clk_Slow); 
  
  
 initial begin
        SSEG_AN <= 8'b11111110;             //start at first anode
    end
	 

always @ (posedge Clk_Slow)
begin 

		 case (faier_counter)
         
				
				 0 : LEDS <= 8'b11111111;
				 1 : LEDS <= 8'b01111111;
				 2 : LEDS <= 8'b00111111;
				 3 : LEDS <= 8'b00011111;
				 4 : LEDS <= 8'b00001111;
				 5 : LEDS <= 8'b00000111;
				 6 : LEDS <= 8'b00000011;
				 7 : LEDS <= 8'b00000001;
				 8 : LEDS <= 8'b00000000;
				 default : LEDS <= 8'b11111111;
        endcase
		  
		  case (killCounter)
         
				
				 0 : SSEG_CA <= 8'b00000010;
				 1 : SSEG_CA <= 8'b10011110;
				 2 : SSEG_CA <= 8'b00100100;
				 3 : SSEG_CA <= 8'b00001100;
				 4 : SSEG_CA <= 8'b10011000;
				 5 : SSEG_CA <= 8'b01001000;
				 6 : SSEG_CA <= 8'b01000000;
				 7 : SSEG_CA <= 8'b00011110;
				 8 : SSEG_CA <= 8'b00000000;
				 9 : SSEG_CA <= 8'b00011000;
				 default : SSEG_CA <= 8'b11111111;
        endcase
		  
		  
		  
		  
		  
				case (SSEG_AN)
            8'b11111110: SSEG_AN <= 8'b11111110;
           // 8'b11111101: SSEG_AN <= 8'b11111011;
           // 8'b11111011: SSEG_AN <= 8'b11111110;
           // 8'b11110111: SSEG_AN <= 8'b11101111;
           // 8'b11101111: SSEG_AN <= 8'b11011111;
           // 8'b11011111: SSEG_AN <= 8'b10111111;
           // 8'b11111101: SSEG_AN <= 8'b01111111;    
            //8'b01111111: SSEG_AN <= 8'b11111110;
        endcase
	


case (killCounter)
				 0 : add <= 150 ;
				 1 : add <= -150 ;
				 2 : add <= 150 ;
				 3 : add <= 243 ;
				 4 : add <= 132 ;
				 5 : add <= -54 ;
				 6 : add <= -23 ;
				 7 : add <= -45 ;
				 8 : add <= 150 ;
				 9 : add <= -50 ;
        endcase
       
		
	
		if(right == 0)
													begin 
													
													right_counter<=right_counter+5;
													
																if(fire == 0)							
																begin
																
																	
																	munition_position_orizont<=right_counter;
																
																end
												
													end
												
													
		if(left == 0)
													begin 
													
													right_counter<=right_counter-5;
														if(fire == 0)							
																begin
																
																
																	munition_position_orizont<=right_counter;
																
																end
													end
												
		if(shot == 0) 
													begin
													
													if(permision == 1)begin
														
														permision <= 0;
													end
													
													
													fire <= 1;
													
													end
													
		if(shot == 1)  						begin
		
													permision <= 1;
		
													end
		
		if(fire == 1)							
													begin
													
													
													munition_position<=munition_position+10;
												
													end
													
		if(resetValues == 0) 						
													begin
													fire<=0;
													munition_position<=0;
													killCounter<=0;
													enemyAltitude<=0;
													faier_counter<=0;
													end
													
		if(killCounter == 9 || faier_counter == 9) 						
													begin
													fire<=0;
													munition_position<=0;
													killCounter<=0;
													enemyAltitude<=0;
													faier_counter<=0;
													end											
		
		if(munition_position == 350) 						
													begin
													fire<=0;
													munition_position<=0;
													munition_position_orizont<=right_counter;
													faier_counter <= faier_counter+1;
													end
													
	
		if (enemyAltitude <= 480)		   begin
																enemyAltitude <= enemyAltitude+5;
													end
													else
															begin
																enemyAltitude<=0;
															end
		if (enemyAltitude >= 350 - munition_position && 300 + add >= 200 + munition_position_orizont && 200 + add <=180 + munition_position_orizont)
													begin
															enemyAltitude<=0;
															fire<=0;
															munition_position<=0;
															munition_position_orizont<=right_counter;
															killCounter <= killCounter+1;
															
													end
													
end

 music M1 (.clk(vga_clk), .speaker(speaker));
 always @(posedge clock) begin
 if(timer_t > 250) begin					// generate 10 uS RESET signal 
		reset <= 0;
	end
	else begin
		reset <= 1;					//while in reset display is disabled, suare is set to initial position
		timer_t <= timer_t + 1;
		disp_en <= 0;			
		sq_pos_x <= init_x;				
		sq_pos_y <= init_y;
	end
if (killCounter<7)begin	
  if (enable == 1)
  begin
    if(hcount == 799)
    begin
      hcount <= 0;
      if(vcount == 524)
        vcount <= 0;
      else 
        vcount <= vcount+1'b1;
    end
    else
      hcount <= hcount+1'b1;
 
 
  if (vcount >= 490 && vcount < 492) 
    vsync <= 1'b0;
  else
    vsync <= 1'b1;

  if (hcount >= 656 && hcount < 752) 
    hsync <= 1'b0;
  else
    hsync <= 1'b1;
  end
end  
else begin

if(c_hor < h_pixels + h_fp + 1 || c_hor > h_pixels + h_fp + h_pulse) begin	// H-SYNC generator
		hsync <= ~h_pol;
	end
	else begin
		hsync <= h_pol;
	end
	if(c_ver < v_pixels + v_fp || c_ver > v_pixels + v_fp + v_pulse) begin		//V-SYNC generator
		vsync <= ~v_pol;
	end
	else begin
		vsync <= v_pol;
	end


 end


  if (enable)
				begin                  
						
												 
												 
											 	 if (hcount < 240 + right_counter && hcount > 140 + right_counter && vcount < 480 && vcount > 400)
												 begin
													green <= 3'b111;
													blue <= 2'b11; 
													red <= 3'b111;
												 end
												  else 
												if (hcount < 210 + right_counter && hcount > 170 + right_counter && vcount < 410 && vcount > 350)  
												 begin
													green <= 3'b111;
													blue <= 2'b11; 
													red <= 3'b000;
												 end
												 else
												 if (hcount < 200 + munition_position_orizont && hcount > 180 + munition_position_orizont && vcount < 410 - munition_position && vcount > 350 - munition_position)  
												 begin
													green <= 3'b111;
													blue <= 2'b11; 
													red <= 3'b000;
												 end
												 else
												 if (hcount < 300 + add && hcount > 200 + add  && vcount < 0 + enemyAltitude  && vcount > -70 + enemyAltitude)  
												 begin
													green <= 3'b000;
													blue <= 2'b00; 
													red <= 3'b111;
												 end
												 else
												 if(killCounter >=7)begin 
													if(reset == 1) begin					//while RESET is high init counters
	
	
		sq_figure[0][19:0] <=	20'b11011011111011000011;
		sq_figure[1][19:0] <=	20'b11011011111011000011;
		sq_figure[2][19:0] <=	20'b11011010011011000011;
		sq_figure[3][19:0] <=	20'b11011010011011000011;
		sq_figure[4][19:0] <=	20'b11011010011011000011;
		sq_figure[5][19:0] <=	20'b11011010011011111111;
		sq_figure[6][19:0] <=	20'b11011010011011000000;
		sq_figure[7][19:0] <=	20'b11011010011011000000;
		sq_figure[8][19:0] <=	20'b11011010011011000000;
		sq_figure[9][19:0] <=	20'b11111011111011000000;
		sq_figure[10][19:0] <=	20'b00000000000000000000;
		sq_figure[11][19:0] <=	20'b11000110111011011011;
		sq_figure[12][19:0] <=	20'b11100110111011011011;
		sq_figure[13][19:0] <=	20'b11100110000011011011;
		sq_figure[14][19:0] <=	20'b11110110111011011011;
		sq_figure[15][19:0] <=	20'b11011110111011011011;
		sq_figure[16][19:0] <=	20'b11001110111011011011;
		sq_figure[17][19:0] <=	20'b11001110111011011011;
		sq_figure[18][19:0] <=	20'b11001110111011111111;
		sq_figure[19][19:0] <=	20'b11000110111011111111;
	
		c_hor <= 0;
		c_ver <= 0;
		hsync <= 1;
		vsync <= 0;
		c_row <= 0;
		c_col <= 0;
	end
	else begin						// update current beam position
		if(c_hor < h_frame - 1) begin
			c_hor <= c_hor + 1;
		end
		else begin
			c_hor <= 0;
			if(c_ver < v_frame - 1) begin
				c_ver <= c_ver + 1;
			end
			else begin
				c_ver <= 0;
			end
		end
	end
	
	if(c_hor < h_pixels) begin		//c_col and c_row counters are updated only in the visible time-frame
		c_col <= c_hor;
	end
	if(c_ver < v_pixels) begin
		c_row <= c_ver;
	end
	if(c_hor < h_pixels && c_ver < v_pixels) begin		//VGA color signals are enabled only in the visible time frame
		disp_en <= 1;
	end
	else begin
		disp_en <= 0;
	end
	if(disp_en == 1 && reset == 0) begin
		 if(c_col > l_sq_pos_x && c_col < r_sq_pos_x && c_row > u_sq_pos_y && c_row < d_sq_pos_y) begin	//generate blue square
			//vga_r_r <= 7;
			//vga_g_r <= 0;
			//vga_b_r <= 7;
			if(sq_figure[sq_fig_y][sq_fig_x] == 1) begin
			red <= 7;
			green <= 0;
			blue <= 7;
			end
			else begin
			red <= 0;
			green <= 0;
			blue <= 0;
			end
		end
		else begin			//everything else is black
			red <= 0;
			green <= 0;
			blue <= 0;
		end
	end
	end
												 else
												 begin 
												   green <= 3'b000;
													blue <= 2'b00; 
													red <= 3'b000;
												 end
												
												 
												 
					

	 end
	  arr_timer <= arr_timer + 1;
 end



endmodule


module slow_clock(CLK, Clk_Slow);
    input CLK;
    output Clk_Slow;

    reg [31:0] counter_out;
    reg Clk_Slow; 

initial begin 
   counter_out<=32'h00000000; 
   Clk_Slow<=0; 
end 

//this always block runs on the fast 100MHz clock

    always @(posedge CLK) begin 
   counter_out<=counter_out + 32'h00000001; 
   if (counter_out>32'h001EBC20) begin 
       counter_out<=32'h00000000; 
       Clk_Slow<=!Clk_Slow; 
   end 
end
endmodule

//--------------------------------Sound--------------------------

// Music demo verilog file
// (c) fpga4fun.com 2003-2015

// Plays a little tune on a speaker
// Use a 25MHz clock if possible (other frequencies will 
// change the pitch/speed of the song)

/////////////////////////////////////////////////////
module music(
	input clk ,
	output reg speaker
);

reg [30:0] tone;
always @(posedge clk) tone <= tone+31'd1;

reg[49:0] count_time = 0;
reg clk_out;

always @(posedge clk)
begin
	count_time <= count_time+1;
	if(count_time==10)
	begin 
	count_time <= 0;
	clk_out = ~clk_out;
	end
end



wire [7:0] fullnote;
music_ROM get_fullnote(.clk(clk_out), .address(tone[29:22]), .note(fullnote));

wire [2:0] octave;
wire [3:0] note;
divide_by12 get_octave_and_note(.numerator(fullnote[5:0]), .quotient(octave), .remainder(note));

reg [8:0] clkdivider;
always @*
case(note)
	 0: clkdivider = 9'd511;//A
	 1: clkdivider = 9'd482;// A#/Bb
	 2: clkdivider = 9'd455;//B
	 3: clkdivider = 9'd430;//C
	 4: clkdivider = 9'd405;// C#/Db
	 5: clkdivider = 9'd383;//D
	 6: clkdivider = 9'd361;// D#/Eb
	 7: clkdivider = 9'd341;//E
	 8: clkdivider = 9'd322;//F
	 9: clkdivider = 9'd303;// F#/Gb
	10: clkdivider = 9'd286;//G
	11: clkdivider = 9'd270;// G#/Ab
	default: clkdivider = 9'd0;
endcase
//slow_clock S1(clk, Clk_Slow);

reg [8:0] counter_note;
reg [7:0] counter_octave;
always @(posedge clk) counter_note <= counter_note==0 ? clkdivider : counter_note-9'd1;
always @(posedge clk) if(counter_note==0) counter_octave <= counter_octave==0 ? 8'd255 >> octave : counter_octave-8'd1;
always @(posedge clk) if(counter_note==0 && counter_octave==0 && fullnote!=0 && tone[21:18]!=0) speaker <= ~speaker;
endmodule


/////////////////////////////////////////////////////
module divide_by12(
	input [5:0] numerator,  // value to be divided by 12
	output reg [2:0] quotient, 
	output [3:0] remainder
);

reg [1:0] remainder3to2;
always @(numerator[5:2])
case(numerator[5:2])
	 0: begin quotient=0; remainder3to2=0; end
	 1: begin quotient=0; remainder3to2=1; end
	 2: begin quotient=0; remainder3to2=2; end
	 3: begin quotient=1; remainder3to2=0; end
	 4: begin quotient=1; remainder3to2=1; end
	 5: begin quotient=1; remainder3to2=2; end
	 6: begin quotient=2; remainder3to2=0; end
	 7: begin quotient=2; remainder3to2=1; end
	 8: begin quotient=2; remainder3to2=2; end
	 9: begin quotient=3; remainder3to2=0; end
	10: begin quotient=3; remainder3to2=1; end
	11: begin quotient=3; remainder3to2=2; end
	12: begin quotient=4; remainder3to2=0; end
	13: begin quotient=4; remainder3to2=1; end
	14: begin quotient=4; remainder3to2=2; end
	15: begin quotient=5; remainder3to2=0; end
endcase

assign remainder[1:0] = numerator[1:0];  // the first 2 bits are copied through
assign remainder[3:2] = remainder3to2;  // and the last 2 bits come from the case statement
endmodule
/////////////////////////////////////////////////////


module music_ROM(
	input clk,
	input [7:0] address,
	output reg [7:0] note
);

always @(posedge clk)
case(address)
	  0: note<= 8'd25;
	  1: note<= 8'd27;
	  2: note<= 8'd27;
	  3: note<= 8'd25;
	  4: note<= 8'd22;
	  5: note<= 8'd22;
	  6: note<= 8'd30;
	  7: note<= 8'd30;
	  8: note<= 8'd27;
	  9: note<= 8'd27;
	 10: note<= 8'd25;
	 11: note<= 8'd25;
	 12: note<= 8'd25;
	 13: note<= 8'd25;
	 14: note<= 8'd25;
	 15: note<= 8'd25;
	 16: note<= 8'd25;
	 17: note<= 8'd27;
	 18: note<= 8'd25;
	 19: note<= 8'd27;
	 20: note<= 8'd25;
	 21: note<= 8'd25;
	 22: note<= 8'd30;
	 23: note<= 8'd30;
	 24: note<= 8'd29;
	 25: note<= 8'd29;
	 26: note<= 8'd29;
	 27: note<= 8'd29;
	 28: note<= 8'd29;
	 29: note<= 8'd29;
	 30: note<= 8'd29;
	 31: note<= 8'd29;
	 32: note<= 8'd23;
	 33: note<= 8'd25;
	 34: note<= 8'd25;
	 35: note<= 8'd23;
	 36: note<= 8'd20;
	 37: note<= 8'd20;
	 38: note<= 8'd29;
	 39: note<= 8'd29;
	 40: note<= 8'd27;
	 41: note<= 8'd27;
	 42: note<= 8'd25;
	 43: note<= 8'd25;
	 44: note<= 8'd25;
	 45: note<= 8'd25;
	 46: note<= 8'd25;
	 47: note<= 8'd25;
	 48: note<= 8'd25;
	 49: note<= 8'd27;
	 50: note<= 8'd25;
	 51: note<= 8'd27;
	 52: note<= 8'd25;
	 53: note<= 8'd25;
	 54: note<= 8'd27;
	 55: note<= 8'd27;
	 56: note<= 8'd22;
	 57: note<= 8'd22;
	 58: note<= 8'd22;
	 59: note<= 8'd22;
	 60: note<= 8'd22;
	 61: note<= 8'd22;
	 62: note<= 8'd22;
	 63: note<= 8'd22;
	 64: note<= 8'd25;
	 65: note<= 8'd27;
	 66: note<= 8'd27;
	 67: note<= 8'd25;
	 68: note<= 8'd22;
	 69: note<= 8'd22;
	 70: note<= 8'd30;
	 71: note<= 8'd30;
	 72: note<= 8'd27;
	 73: note<= 8'd27;
	 74: note<= 8'd25;
	 75: note<= 8'd25;
	 76: note<= 8'd25;
	 77: note<= 8'd25;
	 78: note<= 8'd25;
	 79: note<= 8'd25;
	 80: note<= 8'd25;
	 81: note<= 8'd27;
	 82: note<= 8'd25;
	 83: note<= 8'd27;
	 84: note<= 8'd25;
	 85: note<= 8'd25;
	 86: note<= 8'd30;
	 87: note<= 8'd30;
	 88: note<= 8'd29;
	 89: note<= 8'd29;
	 90: note<= 8'd29;
	 91: note<= 8'd29;
	 92: note<= 8'd29;
	 93: note<= 8'd29;
	 94: note<= 8'd29;
	 95: note<= 8'd29;
	 96: note<= 8'd23;
	 97: note<= 8'd25;
	 98: note<= 8'd25;
	 99: note<= 8'd23;
	100: note<= 8'd20;
	101: note<= 8'd20;
	102: note<= 8'd29;
	103: note<= 8'd29;
	104: note<= 8'd27;
	105: note<= 8'd27;
	106: note<= 8'd25;
	107: note<= 8'd25;
	108: note<= 8'd25;
	109: note<= 8'd25;
	110: note<= 8'd25;
	111: note<= 8'd25;
	112: note<= 8'd25;
	113: note<= 8'd27;
	114: note<= 8'd25;
	115: note<= 8'd27;
	116: note<= 8'd25;
	117: note<= 8'd25;
	118: note<= 8'd32;
	119: note<= 8'd32;
	120: note<= 8'd30;
	121: note<= 8'd30;
	122: note<= 8'd30;
	123: note<= 8'd30;
	124: note<= 8'd30;
	125: note<= 8'd30;
	126: note<= 8'd30;
	127: note<= 8'd30;
	128: note<= 8'd27;
	129: note<= 8'd27;
	130: note<= 8'd27;
	131: note<= 8'd27;
	132: note<= 8'd30;
	133: note<= 8'd30;
	134: note<= 8'd30;
	135: note<= 8'd27;
	136: note<= 8'd25;
	137: note<= 8'd25;
	138: note<= 8'd22;
	139: note<= 8'd22;
	140: note<= 8'd25;
	141: note<= 8'd25;
	142: note<= 8'd25;
	143: note<= 8'd25;
	144: note<= 8'd23;
	145: note<= 8'd23;
	146: note<= 8'd27;
	147: note<= 8'd27;
	148: note<= 8'd25;
	149: note<= 8'd25;
	150: note<= 8'd23;
	151: note<= 8'd23;
	152: note<= 8'd22;
	153: note<= 8'd22;
	154: note<= 8'd22;
	155: note<= 8'd22;
	156: note<= 8'd22;
	157: note<= 8'd22;
	158: note<= 8'd22;
	159: note<= 8'd22;
	160: note<= 8'd20;
	161: note<= 8'd20;
	162: note<= 8'd22;
	163: note<= 8'd22;
	164: note<= 8'd25;
	165: note<= 8'd25;
	166: note<= 8'd27;
	167: note<= 8'd27;
	168: note<= 8'd29;
	169: note<= 8'd29;
	170: note<= 8'd29;
	171: note<= 8'd29;
	172: note<= 8'd29;
	173: note<= 8'd29;
	174: note<= 8'd29;
	175: note<= 8'd29;
	176: note<= 8'd30;
	177: note<= 8'd30;
	178: note<= 8'd30;
	179: note<= 8'd30;
	180: note<= 8'd29;
	181: note<= 8'd29;
	182: note<= 8'd27;
	183: note<= 8'd27;
	184: note<= 8'd25;
	185: note<= 8'd25;
	186: note<= 8'd23;
	187: note<= 8'd20;
	188: note<= 8'd20;
	189: note<= 8'd20;
	190: note<= 8'd20;
	191: note<= 8'd20;
	192: note<= 8'd25;
	193: note<= 8'd27;
	194: note<= 8'd27;
	195: note<= 8'd25;
	196: note<= 8'd22;
	197: note<= 8'd22;
	198: note<= 8'd30;
	199: note<= 8'd30;
	200: note<= 8'd27;
	201: note<= 8'd27;
	202: note<= 8'd25;
	203: note<= 8'd25;
	204: note<= 8'd25;
	205: note<= 8'd25;
	206: note<= 8'd25;
	207: note<= 8'd25;
	208: note<= 8'd25;
	209: note<= 8'd27;
	210: note<= 8'd25;
	211: note<= 8'd27;
	212: note<= 8'd25;
	213: note<= 8'd25;
	214: note<= 8'd30;
	215: note<= 8'd30;
	216: note<= 8'd29;
	217: note<= 8'd29;
	218: note<= 8'd29;
	219: note<= 8'd29;
	220: note<= 8'd29;
	221: note<= 8'd29;
	222: note<= 8'd29;
	223: note<= 8'd29;
	224: note<= 8'd23;
	225: note<= 8'd25;
	226: note<= 8'd25;
	227: note<= 8'd23;
	228: note<= 8'd20;
	229: note<= 8'd20;
	230: note<= 8'd29;
	231: note<= 8'd29;
	232: note<= 8'd27;
	233: note<= 8'd27;
	234: note<= 8'd25;
	235: note<= 8'd25;
	236: note<= 8'd25;
	237: note<= 8'd25;
	238: note<= 8'd25;
	239: note<= 8'd25;
	240: note<= 8'd25;
	241: note<= 8'd0;
	242: note<= 8'd00;
	default: note <= 8'd0;
endcase
endmodule


/////////////////////////////////////////////////////







