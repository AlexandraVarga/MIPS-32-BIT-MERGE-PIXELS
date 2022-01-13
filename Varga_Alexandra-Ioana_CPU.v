`default_nettype none
`default_nettype wire

module processor(
input clk,reset,
output [31:0] PC,
input [31:0] instruction,
output WE,
output [31:0] address_to_mem,
output [31:0] data_to_mem,
input [31:0] data_from_mem


);

wire [31:0] rs1,rs2,WriteD,SignExtInstr,rs2sel_Out,ALUResult,LuiMuxout;
wire [31:0]pcadd,BranchMuxout,JALout,JALrout,PCMuxout,JALMuxout,Branchaddress;
wire JALRSel,Branch,PCSrcJal,RegWrite,Zero_Out,LuiSel;
wire [1:0]ALUSrc, ALUOp, MemtoReg;
wire [2:0] Operation;
wire [4:0] rdaddress;

assign rdaddress=instruction[11:7];

	ProgramCounter pg(
 .D(PCMuxout),
 .clk(clk),
 .reset(reset),
 .Q(PC)
 
 );
 
 addPC ADDPC(
 .InPC(PC),
 .OutPC(pcadd)
 );
 
 Mux32bit_2inp PCMux(
 .A(BranchMuxout),
 .B(JALMuxout),
 .C(PCMuxout),
 .sel(PCSrcJal)
 );
 
Mux32bit_2inp BranchMux(
 .A(pcadd),
 .B(Branchaddress),
 .C(BranchMuxout),
 .sel(Branch)
 );
 
Mux32bit_2inp JalRMux(
 .A(JALout),
 .B(JALrout),
 .C(JALMuxout),
 .sel(JALRSel)
 );
 

JALRCalc JLRCalc(
.rs1(rs1),
.imm(instruction[31:20]),
.Jalrout(JALrout)
);


JALCalc JLCalc(
.instr(instruction),
.PC(PC),
.Jalout(JALout)
);

BranchAddCalc BRAdCalc(
.instr(instruction),
.PC(PC),
.Branchaddress(Branchaddress)
);


//Register Calling
REG Registers(
 .AddrA(instruction[19:15]),
 .AddrB(instruction[24:20]),
 .AddrD(rdaddress),
 .clk(clk),
 .RegWrite(RegWrite),
 .DataD(WriteD),
 .rs1(rs1),
 .rs2(rs2)
);

SignExtend Signextend12bit(
.Instr12(instruction[31:20]),
.Instr32(SignExtInstr)
//output [31:0]Instr32_Shifted
);


 
Mux32bit_3inp ALUSrcMux
(
 .A(rs2),
 .B(SignExtInstr),
 .C({{20{instruction[31]}},instruction[31:25], instruction[11:7]}),
 .D(rs2sel_Out),
 .sel(ALUSrc)
 
);
  
//ALUControls
ALUcontrol ALUCtrl(
.func({instruction[30],instruction[14:12]}),
.ALUOp(ALUOp),
.Operation(Operation)
);



 ControlUnit CU(
.Opcode(instruction[6:0]),
.ZeroALU(Zero_Out),
.JALRSel(JALRSel),
.Branch(Branch),
.PCSrcJal(PCSrcJal),
.MemtoReg(MemtoReg),
.MemWrite(WE),
.ALUSrc(ALUSrc),
.RegWrite(RegWrite),
.LuiSel(LuiSel),
.ALUOp(ALUOp)
);

//ALU
ALU ALU(
.Operation(Operation),
.rs1(rs1),
.rs2(rs2sel_Out),
.rd(ALUResult),
.zero(Zero_Out)
);

//LuiMux

LuiMux LuiMux(
.imm(instruction[31:12]),
.PC(PC),
.sel(LuiSel),
.Luimuxout(LuiMuxout)
);

//MemtoRegMux
Mux32bit_4inp MemtoRegMux(
 .A(ALUResult),
 .B(data_from_mem),
 .C(pcadd),
 .D(LuiMuxout),
 .E(WriteD),
 .sel(MemtoReg)
 );

 
assign address_to_mem=ALUResult;
assign data_to_mem=rs2;

endmodule

module ProgramCounter(
input [31:0]D, 
input clk,reset,
output reg [31:0]Q
);


always @(posedge clk)
begin
	if (reset)
		Q<=0;
	else
		Q<=D;
	
end
endmodule

module addPC(
input [31:0]InPC,
output [31:0]OutPC
);

assign OutPC=InPC + 32'h00000004;

endmodule

module REG
  (
   input [4:0] AddrA,
   input [4:0] AddrB,
	input [4:0] AddrD,
	input clk,
	input RegWrite,
   input [31:0] DataD,
   output [31:0] rs1,
   output [31:0] rs2
	);

   reg [31:0] regfile [31:0];
	
	integer i;	
	initial begin
	for (i=0;i<32;i=i+1)
	begin	
		regfile[i]=32'h00000000;
	end
	end
	
	assign rs1=regfile[AddrA];
	assign rs2=regfile[AddrB];
	

   always @(posedge clk) begin
      
		if (RegWrite & ~(AddrD==0))
	 	    regfile[AddrD]<=DataD;
      else
			 regfile[AddrD]<=regfile[AddrD];
		end
		
endmodule

module ALU(
input [2:0]Operation,
input [31:0] rs1,
input [31:0] rs2,
output reg [31:0] rd,
output zero
);


wire [31:0] addop, subop, andop, sll, slt, srl, sra, adduqb;
wire unsigned [31:0] rs2u;
wire signed [31:0] rs1s,rs2s;

assign addop = rs1 + rs2;
assign subop = rs1 - rs2;
assign rs2u=rs2;
assign rs1s=rs1;
assign rs2s=rs2;
assign andop = rs1 & rs2;
assign sll = rs1 << rs2u;
assign slt = rs1s < rs2s;
assign srl = rs1 >> rs2u;
assign sra = rs1s >>> rs2;
assign adduqb={rs1[31:24]+rs2[31:24],rs1[23:16]+rs2[23:16],rs1[15:8]+rs2[15:8],rs1[7:0]+rs2[7:0]};
assign zero=(subop==0);

	
always @ (*)
begin
if (Operation==3'b000)
	rd=addop;
else if (Operation==3'b001)
	rd=subop;
else if (Operation==3'b010)
	rd=andop;
else if (Operation==3'b011)
	rd=slt;
else if (Operation==3'b100)
	rd=srl;
else if (Operation==3'b101)
	rd=sll;
else if (Operation==3'b110)
	rd=sra;
else
	rd=adduqb;	
end
endmodule

module ALUcontrol(
input [3:0]func,
input [1:0]ALUOp,
output[2:0]Operation
);

assign Operation[2]=~func[3]&func[2]&~func[1]&func[0]&ALUOp[1]&~ALUOp[0]||~func[3]&~func[2]&~func[1]&func[0]&ALUOp[1]&~ALUOp[0]||func[3]&func[2]&~func[1]&func[0]&ALUOp[1]&~ALUOp[0]||ALUOp[1]&ALUOp[0];

assign Operation[1]=~func[3]&func[2]&func[1]&func[0]&ALUOp[1]&~ALUOp[0]||~func[3]&~func[2]&func[1]&~func[0]&ALUOp[1]&~ALUOp[0]||func[3]&func[2]&~func[1]&func[0]&ALUOp[1]&~ALUOp[0]||ALUOp[1]&ALUOp[0];

assign Operation[0]=func[3]&~func[2]&~func[1]&~func[0]&ALUOp[1]&~ALUOp[0]||~func[3]&~func[2]&func[1]&~func[0]&ALUOp[1]&~ALUOp[0]||~func[3]&~func[2]&~func[1]&func[0]&ALUOp[1]&~ALUOp[0]||~ALUOp[1]&ALUOp[0]||ALUOp[1]&ALUOp[0];


endmodule




module ControlUnit(
input [6:0]Opcode,
input ZeroALU,
output JALRSel,Branch,PCSrcJal,
output [1:0] MemtoReg,
output MemWrite,
output [1:0]ALUSrc,
output RegWrite,LuiSel,
output [1:0]ALUOp
);


assign JALRSel=Opcode[6]&Opcode[5]&~Opcode[4]&~Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0];

assign Branch=ZeroALU&Opcode[6]&Opcode[5]&~Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0];

assign PCSrcJal=Opcode[6]&Opcode[5]&~Opcode[4]&~Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0]||Opcode[6]&Opcode[5]&~Opcode[4]&Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0];

assign LuiSel=~Opcode[6]&Opcode[5]&Opcode[4]&~Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0];

assign MemtoReg[1]=PCSrcJal||LuiSel||~Opcode[6]&~Opcode[5]&Opcode[4]&~Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0];

assign MemtoReg[0]=~Opcode[6]&~Opcode[5]&~Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||LuiSel||~Opcode[6]&~Opcode[5]&Opcode[4]&~Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0];

assign MemWrite=~Opcode[6]&Opcode[5]&~Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0];

assign ALUSrc[1]=MemWrite;

assign ALUSrc[0]=~Opcode[6]&~Opcode[5]&~Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||~Opcode[6]&~Opcode[5]&Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0];

assign RegWrite=~Opcode[6]&Opcode[5]&Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||~Opcode[6]&~Opcode[5]&~Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||~Opcode[6]&~Opcode[5]&Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||LuiSel||PCSrcJal||~Opcode[6]&~Opcode[5]&~Opcode[4]&Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||~Opcode[6]&~Opcode[5]&Opcode[4]&~Opcode[3]&Opcode[2]&Opcode[1]&Opcode[0];

assign ALUOp[1]=~Opcode[6]&Opcode[5]&Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||~Opcode[6]&~Opcode[5]&~Opcode[4]&Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0];

assign ALUOp[0]=Opcode[6]&Opcode[5]&~Opcode[4]&~Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0]||~Opcode[6]&~Opcode[5]&~Opcode[4]&Opcode[3]&~Opcode[2]&Opcode[1]&Opcode[0];


endmodule

module Mux32bit_3inp(
input [1:0]sel,
input [31:0]A,
input [31:0]B,
input [31:0]C,
output reg[31:0]D
);

always @(*)
begin
if (sel==2'b00)
	D=A;
else if (sel==2'b01)
	D=B;
else if (sel==2'b10)
	D=C;
else
	D=31'bx;	

end
endmodule

module Mux32bit_2inp(
input sel,
input [31:0]A,
input [31:0]B,
output[31:0]C
);

assign C =  (sel?B:A);

endmodule

module JALCalc(
input [31:0]instr,
input [31:0]PC,
output[31:0]Jalout
);

wire [31:0] signextend;

assign signextend={{11{instr[31]}},instr[31],instr[19:12],instr[20],instr[30:21],1'b0};
assign Jalout=PC+signextend; 

endmodule

module JALRCalc(
input [31:0]rs1,
input [11:0]imm,
output[31:0]Jalrout
);

wire [31:0] signextend;

assign signextend={{20{imm[11]}},imm};
assign Jalrout=rs1+signextend; 

endmodule

module BranchAddCalc(
input [31:0]instr,
input [31:0]PC,
output[31:0]Branchaddress
);

wire [31:0] signextend;

assign signextend={{19{instr[31]}},instr[31],instr[7],instr[30:25],instr[11:8],1'b0};
assign Branchaddress=PC+signextend; 

endmodule

module SignExtend(
input [11:0]Instr12,
output [31:0]Instr32
//output [31:0]Instr32_Shifted
);

assign Instr32={{20{Instr12[11]}},Instr12}; 
//assign Instr32_Shifted=Instr32<<2;
endmodule

module LuiMux (
input [19:0]imm,
input [31:0]PC,
input sel,
output [31:0]Luimuxout
);
 
wire [31:0] Auipcout,Luiout;
assign Auipcout=PC+{imm,12'b0};
assign Luiout={imm,12'b0};

assign Luimuxout =  (sel?Luiout:Auipcout);
 
endmodule
 
module Mux32bit_4inp (
input [31:0]A,B,C,D,
input [1:0]sel,
output reg [31:0]E

);


always @ (*)
begin
if (sel==2'b00)
	E=A;
else if (sel==2'b01)
	E=B;
else if (sel==2'b10)
	E=C;
else
	E=D;
end
endmodule

//---------------------------------------------------------------------------------------------
/*module top (    input         clk, reset,
        output [31:0] data_to_mem, address_to_mem,
        output        write_enable);

    wire [31:0] pc, instruction, data_from_mem;

    inst_mem  imem(pc[7:2], instruction);
    data_mem  dmem(clk, write_enable, address_to_mem, data_to_mem, data_from_mem);
    processor CPU(clk, reset, pc, instruction, write_enable, address_to_mem, data_to_mem, data_from_mem);
    
    always @(posedge clk) $display("PC = %d, instruction = %x, reset = %d", pc, instruction, reset);
endmodule

//-------------------------------------------------------------------
module data_mem (input clk, we,
         input  [31:0] address, wd,
         output [31:0] rd);

    reg [31:0] RAM[63:0];

    initial begin
        $readmemh ("data_V0.hex",RAM,0,63);
    end

    assign rd=RAM[address[31:2]]; // word aligned
    integer i;
    always @ (posedge clk) begin
        if (we)
            RAM[address[31:2]]<=wd;
        //THIS WILL PRINT OUT ENTIRE MEMORY ON EACH CLOCK TICK
            $display("----");
            for(i = 0; i < 16; i = i + 1) begin
                    $display ("%d %h %d %h %d %h %d %h", i, RAM[i], i+16, RAM[i+16], i+32, RAM[i+32], i+48, RAM[i+48]);
            end
    
    end

endmodule

//-------------------------------------------------------------------
module inst_mem (input  [5:0]  address,
         output [31:0] rd);

    reg [31:0] RAM[63:0];
    initial begin
        $readmemh ("meminstr_V2.hex",RAM,0,63);//test/Tichy_Jakub_prog1.hex
    end
    assign rd=RAM[address]; // word aligned
endmodule


module testbench();
    reg         clk;
    reg         reset;
    wire [31:0] data_to_mem, address_to_mem;
    wire        memwrite;
	wire write_enable;

    top simulated_system (clk, reset, data_to_mem, address_to_mem, write_enable);

    initial    begin
        $dumpfile("test-vystup");
        $dumpvars;
        reset<=1; # 2; reset<=0;
        #400; $finish;
    end

    // generate clock
    always    begin
        clk<=1; # 1; clk<=0; # 1;
    end

endmodule*/
