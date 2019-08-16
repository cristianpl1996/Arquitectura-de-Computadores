`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:21:58 08/05/2019 
// Design Name: 
// Module Name:    Processor 
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
module Processor(
    input clk,
    input Reset
    );
//Program Counter	
 
	reg [63:0] PC;
	
//Connections - Fetch
	 
	wire [31:0] Instruction_wire;
	
//Connections - Decode

	wire [4:0] rd_wire = Instruction_wire[4:0];
	wire [4:0] rn_wire = Instruction_wire[9:5];
	wire [4:0] rm_wire = Instruction_wire[20:16];
	wire [4:0] MUXOut1_wire;
	wire [63:0] ReadDataOut1_wire;
	wire [63:0] ReadDataOut2_wire;
	wire Reg2Loc_wire;
	wire ALUSrc_wire; 
	wire Branch_wire;
	wire [2:0]ALUOp_wire;	
	wire MemRead_wire; 
	wire MemWrite_wire; 
	wire MemtoReg_wire; 
	wire RegWrite_wire;
	wire Uncondbranch_wire;
	wire [63:0] SignExtOut_wire;	
	wire [63:0] LL2Out_wire;
	wire [63:0] AdderOut_wire;
	wire [63:0] MUXOut2_wire;

//Connections - Execute	

	wire [63:0] ALUResult_wire;
	wire Zero_wire;
	wire ANDOut_wire;
	wire OROut_wire;
	wire [63:0] MUXOut3_wire;
	wire [63:0] DataReadOut_wire;


//Stage - Fetch

	always @(posedge clk)
		if	(PC === 64'bx)
			PC <= 64'b0;
		else if (OROut_wire === 1'b1)//PCSrc
			PC <= AdderOut_wire;//Jump_PC
		else
			PC <= PC+64'b100;
	
	InstructionMemory InstructionMemory (PC[9:2], Instruction_wire);

//Stage - Decode

	Multiplexer2_1_5Bits Multiplexer1 (rm_wire, rd_wire, Reg2Loc_wire, MUXOut1_wire);
	RegisterFile RegisterFile (rn_wire, MUXOut1_wire, rd_wire, MUXOut3_wire, RegWrite_wire, clk, ReadDataOut1_wire, ReadDataOut2_wire);
	ControlUnit ControlUnit (Instruction_wire[31:21], Reg2Loc_wire, ALUSrc_wire, ALUOp_wire, Branch_wire, MemRead_wire, MemWrite_wire, MemtoReg_wire, RegWrite_wire, Uncondbranch_wire);
	SignExtend SignExtend (Instruction_wire, SignExtOut_wire);
	ShiftLeft2 ShiftLeft2 (SignExtOut_wire, LL2Out_wire);
	Adder Adder(PC, LL2Out_wire, AdderOut_wire);
	Multiplexer2_1_64Bits Multiplexer2(ReadDataOut2_wire, SignExtOut_wire, ALUSrc_wire, MUXOut2_wire);
	
//Stage - Execute
	
	ArithmeticLogicUnit ArithmeticLogicUnit(ReadDataOut1_wire, MUXOut2_wire, ALUOp_wire, ALUResult_wire, Zero_wire);
	And And (Branch_wire, Zero_wire, ANDOut_wire);
	Or Or(Uncondbranch_wire, ANDOut_wire, OROut_wire);
	DataMemory DataMemory(ALUResult_wire, ReadDataOut2_wire, MemRead_wire, MemWrite_wire, DataReadOut_wire);
	Multiplexer2_1_64Bits Multiplexer3(ALUResult_wire, DataReadOut_wire, MemtoReg_wire, MUXOut3_wire);
	
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
			DataRead <= 64'b0;			
endmodule
