`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   11:57:25 08/05/2019
// Design Name:   Processor
// Module Name:   C:/Users/Cristian/Documents/ISE/MonoCycle/Processor_TF.v
// Project Name:  MonoCycle
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: Processor
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module Processor_TF;

	// Inputs
	reg clk;
	reg Reset;

	// Instantiate the Unit Under Test (UUT)
	Processor uut (
		.clk(clk), 
		.Reset(Reset)
	);

	parameter PERIOD = 200;
	
	initial 
		begin
			clk = 1;
			forever
			#(PERIOD/2) clk = ~clk;
		end
	initial
		begin
			Reset = 1;
			#(PERIOD/2);
			Reset = 0;
			#(PERIOD/2);
		end	         
endmodule

