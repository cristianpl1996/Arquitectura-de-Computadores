`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:30:25 02/23/2019 
// Design Name:    ARM LEGv8 
// Module Name:    Processor 
// Project Name:   Design ARM Architecture
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
module Processor(
    input clk,
    input Reset		
    );

//Program Counter	
 
	reg [63:0] PC;

//Connections 1 - Instruction Fetch(IF). 

	wire [31:0] Instruction;

//	Connections 1/2 - IF/ID Registers.
 
	wire [63:0] PC_ID;
	wire [31:0] Instruction_ID;
	
//Connections 2 - Instruction Decode(ID).

	wire Reg2Loc;
	wire ALUSrc;
	wire [2:0] ALUOp; 
	wire Branch; 
	wire MemRead; 
	wire MemWrite; 
	wire MemtoReg; 
	wire RegWrite;
	wire Uncondbranch;
	wire [4:0] MUXOut1;
	wire [63:0] ReadDataOut1;
	wire [63:0] ReadDataOut2;
	wire [63:0] SignExtOut;
	wire [4:0] rd_ID = Instruction_ID[4:0];
	wire [4:0] rn_ID = Instruction_ID[9:5];
	wire [4:0] rm_ID = Instruction_ID[20:16];
	wire ALUSrc_out;
	wire [2:0] ALUOp_out; 
	wire Branch_out; 
	wire MemRead_out; 
	wire MemWrite_out; 
	wire MemtoReg_out; 
	wire RegWrite_out;
	wire Uncondbranch_out;
	wire stall;

//	Connections 2/3 - ID/EX Registers.

	wire [63:0] PC_EX;
	wire ALUSrc_EX;
	wire [2:0] ALUOp_EX; 
	wire Branch_EX; 
	wire MemRead_EX; 
	wire MemWrite_EX; 
	wire MemtoReg_EX; 
	wire RegWrite_EX;
	wire Uncondbranch_EX;
	wire [63:0] ReadDataOut1_EX;
	wire [63:0] ReadDataOut2_EX;
	wire [63:0] SignExtOut_EX;
	wire [4:0] rd_EX;
	wire [4:0] rn_EX;
	wire [4:0] rm_EX;
 
//Connections 3 - Execution(EX).

	wire [63:0] LL2Out;
	wire [63:0] MUXOut2;
	wire [63:0] AdderOut;
	wire Zero;
	wire [63:0] ALUResult;
	wire [1:0] ForwardA; 
	wire [63:0] MUXOut5;
	wire [1:0] ForwardB; 
	wire [63:0] MUXOut6;

//	Connections 3/4 - EX/MEM Registers.

	wire Branch_MEM; 
	wire MemRead_MEM; 
	wire MemWrite_MEM; 
	wire MemtoReg_MEM; 
	wire RegWrite_MEM;
	wire Uncondbranch_MEM;
	wire [63:0] MUXOut6_MEM;
	wire [4:0] rd_MEM;
	wire [63:0] AdderOut_MEM;
	wire Zero_MEM;
	wire [63:0]	ALUResult_MEM;

//Connections 4 - Data memory(MEM).

	wire ANDOut;
	wire OROut;
	wire [63:0] DataReadOut;

//	Connections 4/5 - MEM/WB Registers.

	wire RegWrite_WB;
	wire MemtoReg_WB;
	wire [63:0]	ALUResult_WB;
	wire [63:0] DataReadOut_WB;
	wire [4:0] rd_WB;

//Connections 5 - Write back(WB).

	wire [63:0] MUXOut4;

//Stage 1 - Instruction Fetch (IF). 

	always @(posedge clk)
		if (stall !== 1'b1)
			if	(PC === 64'bx)
				PC <= 64'b0;
			else if (OROut === 1'b1)//PCSrc
				PC <= AdderOut_MEM;//Jump_PC
			else
				PC <= PC+64'b100;
				
	InstructionMemory InstructionMemory (PC[9:2], Instruction);
	 
//	Stage 1/2 - IF/ID Registers.

	IF_ID RegistersDecode (clk, Reset, PC, Instruction, PC_ID, Instruction_ID);	
	
//Stage 2 - Instruction Decode(ID).
		
	Multiplexer2_1_5Bits Multiplexer1 (rm_ID, rd_ID, Reg2Loc, MUXOut1);
	RegisterFile RegisterFile (rn_ID, MUXOut1, rd_WB, MUXOut4, RegWrite_WB, clk, ReadDataOut1, ReadDataOut2);
	HarzardDetectionUnit HarzardDetectionUnit (MemRead_EX, rd_EX, rn_ID, rm_ID, stall);	 
	ControlUnit ControlUnit (Instruction_ID[31:21], Reg2Loc, ALUSrc, ALUOp, Branch, MemRead, MemWrite, MemtoReg, RegWrite, Uncondbranch);
	MultiplexerControl MultiplexerControl (stall , ALUSrc, ALUOp, Branch, MemRead, MemWrite, MemtoReg, RegWrite, Uncondbranch, ALUSrc_out, ALUOp_out, Branch_out, MemRead_out, MemWrite_out, MemtoReg_out, RegWrite_out, Uncondbranch_out);	 
	SignExtend SignExtend (Instruction_ID, SignExtOut);

//	Stage 2/3 - ID/EX Registers.

	ID_EX RegistersExecute (clk, Reset, ALUSrc_out, ALUOp_out, Branch_out, MemRead_out, MemWrite_out, MemtoReg_out, RegWrite_out, Uncondbranch_out, PC_ID, ReadDataOut1, ReadDataOut2, SignExtOut, rd_ID, rn_ID, rm_ID, ALUSrc_EX, ALUOp_EX, Branch_EX, MemRead_EX, MemWrite_EX, MemtoReg_EX, RegWrite_EX, Uncondbranch_EX, PC_EX, ReadDataOut1_EX, ReadDataOut2_EX, SignExtOut_EX, rd_EX, rn_EX, rm_EX);	

//Stage 3 - Execution(EX).

	ShiftLeft2 Shiftleft2 (SignExtOut_EX, LL2Out);
	Adder Adder (PC_EX, LL2Out, AdderOut);
	Multiplexer3_1_64Bits Multiplexer5 (ReadDataOut1_EX, MUXOut4, ALUResult_MEM, ForwardA, MUXOut5);
	Multiplexer3_1_64Bits Multiplexer6 (ReadDataOut2_EX, MUXOut4, ALUResult_MEM, ForwardB, MUXOut6);
	Multiplexer2_1_64Bits Multiplexer2 (MUXOut6, SignExtOut_EX, ALUSrc_EX, MUXOut2);
	ArithmeticLogicUnit ArithmeticLogicUnit(MUXOut5, MUXOut2, ALUOp_EX, ALUResult, Zero);
	ForwardingUnit ForwardinUnit (rn_EX, rm_EX, rd_MEM, rd_WB, RegWrite_MEM, RegWrite_WB, ForwardA, ForwardB);

//	Stage 3/4 - EX/MEM Registers.

	EX_MEM RegistersMemory(clk, Reset, Branch_EX, MemRead_EX, MemWrite_EX, MemtoReg_EX, RegWrite_EX, Uncondbranch_EX, AdderOut, Zero, ALUResult, MUXOut6, rd_EX, Branch_MEM, MemRead_MEM, MemWrite_MEM, MemtoReg_MEM, RegWrite_MEM, Uncondbranch_MEM, AdderOut_MEM, Zero_MEM, ALUResult_MEM, MUXOut6_MEM, rd_MEM);

//Stage 4 - Data memory(MEM).

	And And (Branch_MEM, Zero_MEM, ANDOut);
	Or Or (Uncondbranch_MEM, ANDOut, OROut);	 
	DataMemory DataMemory (ALUResult_MEM, MUXOut6_MEM, MemRead_MEM, MemWrite_MEM, DataReadOut);
	
//	Stage 4/5 - MEM/WB Registers.

	MEM_WB RegistersWriteBack (clk, Reset, RegWrite_MEM, MemtoReg_MEM, DataReadOut, ALUResult_MEM, rd_MEM, RegWrite_WB, MemtoReg_WB, DataReadOut_WB, ALUResult_WB, rd_WB);

//Stage 5 - Write back(WB).

	Multiplexer2_1_64Bits Multiplexer4 (ALUResult_WB, DataReadOut_WB, MemtoReg_WB, MUXOut4);
		 
endmodule

//Modules.
	
module InstructionMemory(
    input [7:0] Address,
    output reg[31:0] Instruction
    );
	(* RAM_STYLE = "BLOCK" *) 
	reg [31:0] instr_mem [255:0];
	initial 
		$readmemb ("memory/instruction.dat", instr_mem);
	always @(Address)
		Instruction <= instr_mem[Address];
endmodule

module IF_ID(
	 input clk,
    input Reset,
	 input [63:0] PC,
	input [31:0] Instruction,
    output reg[63:0] PC_ID,	 
	output reg[31:0] Instruction_ID
    );	 
always@(posedge clk)
		if (Reset)
			begin
				PC_ID <= 64'b0;
				Instruction_ID <= 64'b0;
			end
		else
			begin
				PC_ID <= PC;
				Instruction_ID <= Instruction;
			end
endmodule

module HarzardDetectionUnit(
	 input MemRead_EX,
	 input [4:0] rd_EX,
	 input [4:0] rn_ID,
	 input [4:0] rm_ID,
	 output reg stall
    );
	 always @(MemRead_EX, rd_EX, rn_ID, rm_ID) 
		if (MemRead_EX  && ((rd_EX === rn_ID) || (rd_EX === rm_ID))) 
			stall <= 1'b1;
		else 
			stall <= 1'b0;
endmodule

// Constantes Module ControlUnit

// R-Type
`define OP_ADD  11'b10001011000
`define OP_SUB  11'b11001011000
`define OP_AND  11'b10001010000
`define OP_OR   11'b10101010000

// D-Type
`define OP_STUR 11'b11111000000
`define OP_LDUR 11'b11111000010

// I-Type
`define OP_ADDI 11'b1001000100x
`define OP_SUBI 11'b1101000100x

// CB-Type
`define OP_CBZ 11'b10110100xxx

// B-Type
`define OP_B 11'b000101xxxxx

module ControlUnit(
    input [10:0] Opcode,
    output reg Reg2Loc,
    output reg ALUSrc,
    output reg [2:0] ALUOp,
    output reg Branch,
    output reg MemRead,
    output reg MemWrite,
    output reg MemtoReg,
	 output reg RegWrite,
	 output reg Uncondbranch
    );
	always @(Opcode)
			casex (Opcode)
				`OP_ADD: begin 
								Reg2Loc <= 1'b0;
								ALUSrc <= 1'b0;
								Branch <= 1'b0;
								MemRead  <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b000;
							end
				`OP_SUB: begin 
								Reg2Loc <= 1'b0;
								ALUSrc <= 1'b0;
								Branch <= 1'b0;
								MemRead <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b001;
							end
				`OP_AND: begin 
								Reg2Loc <= 1'b0;
								ALUSrc <= 1'b0;
								Branch <= 1'b0;
								MemRead <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b010;
							end
				`OP_OR: begin 
								Reg2Loc <= 1'b0;
								ALUSrc <= 1'b0;
								Branch <= 1'b0;
								MemRead <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b011;
							end
				`OP_STUR: begin 
								Reg2Loc <= 1'b1;
								ALUSrc <= 1'b1;
								Branch <= 1'b0;
								MemRead <= 1'b0;
								MemWrite <= 1'b1;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b0;
								Uncondbranch <= 1'b0;
								ALUOp  <= 3'b000;
							 end
				`OP_LDUR: begin 
								Reg2Loc <= 1'b0;
								ALUSrc <= 1'b1;
								Branch <= 1'b0;
								MemRead <= 1'b1;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b1;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b000;
							 end
				
				`OP_ADDI: begin 
								Reg2Loc <= 1'b1;
								ALUSrc <= 1'b1;
								Branch <= 1'b0;
								MemRead <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b000;
							 end
				`OP_SUBI: begin 
								Reg2Loc <= 1'b1;
								ALUSrc <= 1'b1;
								Branch <= 1'b0;
								MemRead <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b1;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b001;
							 end
				`OP_CBZ: begin 
								Reg2Loc <= 1'b1;
								ALUSrc <= 1'b0;
								Branch <= 1'b1;
								MemRead <= 1'b0;
								MemWrite <= 1'b0;
								MemtoReg <= 1'b0;
								RegWrite <= 1'b0;
								Uncondbranch <= 1'b0;
								ALUOp <= 3'b100;
							end
				`OP_B: begin 
							Reg2Loc <= 1'b1;
							ALUSrc <= 1'b0;
							Branch <= 1'b0;
							MemRead <= 1'b0;
							MemWrite <= 1'b0;
							MemtoReg <= 1'b0;
							RegWrite <= 1'b0;
							Uncondbranch <= 1'b1;
							ALUOp <= 3'b100;
						end
				default:
						begin 
							Reg2Loc <= 1'b0;
							ALUSrc <= 1'b0;
							Branch <= 1'b0;
							MemRead <= 1'b0;
							MemWrite <= 1'b0;
							MemtoReg <= 1'b0;
							RegWrite <= 1'b0;
							Uncondbranch <= 1'b0;
							ALUOp <= 3'b000;
						end
			endcase				
endmodule

module MultiplexerControl(
	 input MuxControl,
    input ALUSrc,
    input [2:0] ALUOp,
    input Branch,
    input MemRead,
    input MemWrite,
    input MemtoReg,
	 input RegWrite,
	 input Uncondbranch,
    output reg ALUSrc_out,
    output reg [2:0] ALUOp_out,
    output reg Branch_out,
    output reg MemRead_out,
    output reg MemWrite_out,
    output reg MemtoReg_out,
	 output reg RegWrite_out,
	 output reg Uncondbranch_out
    );
	 always @(MuxControl, ALUSrc, ALUOp, Branch, MemRead, MemWrite, MemtoReg, RegWrite, Uncondbranch)
	 case (MuxControl)
		 1'b0:begin
					ALUSrc_out <= ALUSrc;
					Branch_out <= Branch;
					MemRead_out <= MemRead;
					MemWrite_out <= MemWrite;
					MemtoReg_out <= MemtoReg;
					RegWrite_out <= RegWrite;
					Uncondbranch_out <= Uncondbranch;
					ALUOp_out <= ALUOp;
				end				
		 default:begin
						ALUSrc_out <= 1'b0;
						Branch_out <= 1'b0;
						MemRead_out <= 1'b0;
						MemWrite_out <= 1'b0;
						MemtoReg_out <= 1'b0;
						RegWrite_out <= 1'b0;
						Uncondbranch_out <= 1'b0;
						ALUOp_out <= 3'b000;
					end
	 endcase
endmodule

module Multiplexer2_1_5Bits(
    input [4:0] A,
    input [4:0] B,
    input S,
    output reg [4:0] Out
    );
	always @(A, B, S)
		case (S)
			1'b0: Out <= A;
			default: Out <= B;
		endcase
endmodule

module RegisterFile(
    input [4:0] ReadReg1,
    input [4:0] ReadReg2,
    input [4:0] WriteReg,
    input [63:0] WriteData,
    input RegWrite,
	 input clk,
    output [63:0] ReadData1,
    output [63:0] ReadData2
    );
	(* RAM_STYLE = "BLOCK" *) 	
	reg [63:0] RF [31:0];
	initial 
		$readmemb("memory/register.dat", RF);
	assign ReadData1 = RF[ReadReg1];
	assign ReadData2 = RF[ReadReg2];
	always @(posedge clk)
		if (RegWrite)
			RF[WriteReg] <= WriteData;	
endmodule

// Constantes Module SignExtend

// R-Type
`define ADD_OP  11'b10001011000
`define SUB_OP  11'b11001011000
`define AND_OP  11'b10001010000
`define OR_OP   11'b10101010000

// D-Type
`define STUR_OP 11'b11111000000
`define LDUR_OP 11'b11111000010

// I-Type
`define ADDI_OP 10'b1001000100
`define SUBI_OP 10'b1101000100

// CB-Type
`define CBZ_OP 8'b10110100

// B-Type
`define B_OP 6'b000101

module SignExtend(
    input [31:0] Instruction,
    output reg[63:0] S
    );
	wire [11:0] ALU_immediate = Instruction[21:10];
	wire [8:0] DT_address = Instruction[20:12];
	wire [25:0] BR_address = Instruction[25:0];
	wire [18:0] COND_BR_address = Instruction[23:5];
	always @(*)
		if	((Instruction[31:22] == `ADDI_OP) || (Instruction[31:22] == `SUBI_OP))
			S <= {52'b0, ALU_immediate};
		else if ((Instruction[31:21] == `STUR_OP) || (Instruction[31:21] == `LDUR_OP)) 
			S <= {{55{DT_address[8]}}, DT_address};
		else if (Instruction[31:26] == `B_OP)
			S <= {{36{BR_address[25]}}, BR_address, 2'b0};
		else if (Instruction[31:24] == `CBZ_OP)
			S <= {{43{COND_BR_address[18]}}, COND_BR_address, 2'b0};
		else
			S <= {{32{Instruction[31]}},Instruction};
endmodule

module ID_EX(
	 input clk,
    input Reset,
    input ALUSrc,
	 input[2:0] ALUOp,
	 input Branch,
    input MemRead,
    input MemWrite,
    input MemtoReg,
	 input RegWrite,
	 input Uncondbranch,
	 input [63:0] PC_ID,
	 input [63:0] ReadDataOut1,
	 input [63:0] ReadDataOut2,
	 input [63:0] SignExtOut,
	 input [4:0] rd_ID,
	 input [4:0] rn_ID,
	 input [4:0] rm_ID,
	 output reg ALUSrc_EX,
    output reg [2:0] ALUOp_EX,
    output reg Branch_EX,
    output reg MemRead_EX,
    output reg MemWrite_EX,
    output reg MemtoReg_EX,
	 output reg RegWrite_EX,
	 output reg Uncondbranch_EX,
	 output reg [63:0] PC_EX,
	 output reg [63:0] ReadDataOut1_EX,
	 output reg [63:0] ReadDataOut2_EX,
	 output reg [63:0] SignExtOut_EX,
	 output reg [4:0] rd_EX,
	 output reg [4:0] rn_EX,
	 output reg [4:0] rm_EX
    );
	 always@(posedge clk)
		if (Reset)
			begin
				ALUSrc_EX <= 1'b0;
				ALUOp_EX <= 1'b0;
				Branch_EX <= 1'b0;
				MemRead_EX <= 1'b0;
				MemWrite_EX <= 1'b0;
				MemtoReg_EX <= 1'b0;
				RegWrite_EX <= 1'b0;
				Uncondbranch_EX <= 1'b0;
				PC_EX <= 64'b0;
				ReadDataOut1_EX <= 64'b0;
				ReadDataOut2_EX <= 64'b0;
				SignExtOut_EX <= 64'b0;
				rd_EX <= 5'b0;
				rn_EX <= 5'b0;
				rm_EX <= 5'b0;
			end
		else
			begin
				ALUSrc_EX <= ALUSrc;
				ALUOp_EX <= ALUOp;
				Branch_EX <= Branch;
				MemRead_EX <= MemRead;
				MemWrite_EX <= MemWrite;
				MemtoReg_EX <= MemtoReg;
				RegWrite_EX <= RegWrite;
				Uncondbranch_EX <= Uncondbranch;
				PC_EX <= PC_ID;
				ReadDataOut1_EX <= ReadDataOut1;
				ReadDataOut2_EX <= ReadDataOut2;
				SignExtOut_EX <= SignExtOut;
				rd_EX <= rd_ID;
				rn_EX <= rn_ID;
				rm_EX <= rm_ID;
			end
endmodule			
			
module Multiplexer3_1_64Bits(
    input [63:0] A,
    input [63:0] B,
    input [63:0] C,
    input [1:0] S,
    output reg [63:0] Out
    );
	 always @(A, B, C, S)
		case (S)
			2'b00: Out <= A;
			2'b01: Out <= B;
			2'b10: Out <= C;
			default: Out <= 64'b0;
		endcase
endmodule

module Multiplexer2_1_64Bits(
    input [63:0] A,
    input [63:0] B,
    input S,
    output reg [63:0] Out
    );
	always @(A, B, S)
		case (S)
			1'b0: Out <= A;
			default: Out <= B;
		endcase
endmodule

module ShiftLeft2(
    input [63:0] A,
    output reg[63:0] S
    );
	 always @(A)
		S <= A << 2;
endmodule

module Adder(
    input [63:0] A,
    input [63:0] B,
    output reg[63:0] Out
    );
	always @(A, B)	
		Out <= A + B;
endmodule

//Constante Module ForwardingUnit

`define thirty_one  5'b11111

module ForwardingUnit(
	 input [4:0] rn_EX,
	 input [4:0] rm_EX,
	 input [4:0] rd_MEM,
	 input [4:0] rd_WB,
	 input RegWrite_MEM,
	 input RegWrite_WB,
	 output reg [1:0] ForwardA,
	 output reg [1:0] ForwardB
    );
	 always @(rn_EX, rm_EX, rd_MEM, rd_WB, RegWrite_MEM, RegWrite_WB) 
		begin
		//2. MEM hazard.		
			if ((RegWrite_WB &&(rd_WB !== `thirty_one)) && /*(!(RegWrite_MEM && (rd_MEM !== `thirty_one) && (rd_MEM !== rn_EX))) &&*/ (rd_WB === rn_EX)) 
				ForwardA <= 2'b01;
		//1. EX hazard.
			else if (RegWrite_MEM && (rd_MEM !== `thirty_one) && (rd_MEM === rn_EX)) 
				ForwardA <= 2'b10;
			else 
				ForwardA <= 2'b00;
		
		//2. MEM hazard.			
			if ((RegWrite_WB && (rd_WB !== `thirty_one)) && /*(!(RegWrite_MEM && (rd_MEM !== `thirty_one) && (rd_MEM !== rm_EX))) &&*/ (rd_WB === rm_EX)) 
				ForwardB <= 2'b01;		
		//1. EX hazard.
			else if (RegWrite_MEM && (rd_MEM !== `thirty_one) && (rd_MEM === rm_EX)) 
				ForwardB <= 2'b10;
			else 
				ForwardB <= 2'b00;	
		end
endmodule

// Constantes Module ArithmeticLogicUnit

`define ADD     3'b000
`define SUB     3'b001
`define AND     3'b010
`define OR      3'b011
`define PassB   3'b100

module ArithmeticLogicUnit(
    input [63:0] A,
    input [63:0] B,
    input [2:0] ALUOp,
    output reg [63:0] ALUResult,
    output Zero
    );
	assign Zero = (ALUResult == 0);
	always @(ALUOp, A, B)
		case (ALUOp)
		   `ADD: ALUResult <= A+B;
			`SUB: ALUResult <= A-B;
			`AND: ALUResult <= A&B;
			`OR: ALUResult <= A|B;
			`PassB: ALUResult <= B;
			default: ALUResult <= 64'b0;
		endcase
endmodule

module EX_MEM(
	 input clk,
    input Reset,
	 input Branch_EX,
    input MemRead_EX,
    input MemWrite_EX,
    input MemtoReg_EX,
	 input RegWrite_EX,
	 input Uncondbranch_EX,
	 input [63:0] AdderOut2,
	 input Zero,
    input [63:0] ALUResult,
	 input [63:0] MUXOut6_EX,
	 input [4:0] rd_EX,
	 output reg Branch_MEM,
    output reg MemRead_MEM,
    output reg MemWrite_MEM,
    output reg MemtoReg_MEM,
	 output reg RegWrite_MEM,
	 output reg Uncondbranch_MEM,
	 output reg [63:0] AdderOut2_MEM,
	 output reg Zero_MEM,
    output reg [63:0] ALUResult_MEM,
	 output reg [63:0] MUXOut6_MEM,
	 output reg [4:0] rd_MEM
    );
	 always@(posedge clk)
		if(Reset)
			begin
				Branch_MEM <= 1'b0;
				MemRead_MEM <= 1'b0;
				MemWrite_MEM <= 1'b0;
				MemtoReg_MEM <= 1'b0;
				RegWrite_MEM <= 1'b0;
				Uncondbranch_MEM <= 1'b0;
				AdderOut2_MEM <= 64'b0;
				Zero_MEM <= 1'b0;
				ALUResult_MEM <= 64'b0;
				MUXOut6_MEM <= 64'b0;
				rd_MEM <= 5'b0;
			end
		else
			begin
				Branch_MEM <= Branch_EX;
				MemRead_MEM <= MemRead_EX;
				MemWrite_MEM <= MemWrite_EX;
				MemtoReg_MEM <= MemtoReg_EX;
				RegWrite_MEM <= RegWrite_EX;
				Uncondbranch_MEM <= Uncondbranch_EX;
				AdderOut2_MEM <= AdderOut2;
				Zero_MEM <= Zero;
				ALUResult_MEM <= ALUResult;
				MUXOut6_MEM <= MUXOut6_EX;
				rd_MEM <= rd_EX;
			end
endmodule

module And(
    input A,
    input B,
    output reg S
    );
	always @(A, B)
		S <= A & B;
endmodule

module Or(
    input A,
    input B,
    output reg S
    );
	always @(A, B)
		S <= A | B;
endmodule

module DataMemory(
	input [63:0] Address,
    input [63:0] DataWrite,
    input MemRead,
    input MemWrite,
    output reg [63:0] DataRead
    );
	(* RAM_STYLE = "BLOCK" *) 
	reg [7:0] data_mem [255:0];
	initial 
		$readmemb("memory/memory.dat", data_mem);
	always @(Address, DataWrite, MemRead, MemWrite)	
		if (MemRead)			
			DataRead <= {data_mem[Address], data_mem[Address+1], data_mem[Address+2], data_mem[Address+3], data_mem[Address+4], data_mem[Address+5], data_mem[Address+6], data_mem[Address+7]};
		else if (MemWrite)
			begin
				data_mem[Address] <= DataWrite[7:0];
				data_mem[Address+1]	<= DataWrite[15:8];
				data_mem[Address+2]	<= DataWrite[23:16];
				data_mem[Address+3]	<= DataWrite[31:24];
				data_mem[Address+4]	<= DataWrite[39:32];
				data_mem[Address+5]	<= DataWrite[47:40];
				data_mem[Address+6]	<= DataWrite[55:48];
				data_mem[Address+7]	<= DataWrite[63:56];
			end
		else
			DataRead <=64'b0;		
endmodule

module MEM_WB(
	 input clk,
    input Reset,
	 input RegWrite_MEM,
	 input MemtoReg_MEM,
	 input [63:0] DataReadOut,
	 input [63:0] ALUResult_MEM,
	 input [4:0] rd_MEM,
	 output reg RegWrite_WB,
	 output reg MemtoReg_WB,
	 output reg [63:0] DataReadOut_WB,
	 output reg [63:0] ALUResult_WB,
	 output reg [4:0] rd_WB
    );
	 always@(posedge clk)
		if(Reset)
			begin
				MemtoReg_WB <= 1'b0;
				RegWrite_WB <= 1'b0;
				DataReadOut_WB <= 64'b0;
				ALUResult_WB <= 64'b0;
				rd_WB <= 5'b0;
			end
		else
			begin
				MemtoReg_WB <= MemtoReg_MEM;
				RegWrite_WB <= RegWrite_MEM;
				DataReadOut_WB <= DataReadOut;
				ALUResult_WB <= ALUResult_MEM;
				rd_WB <= rd_MEM;
			end
endmodule
