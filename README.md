# RegisterFileCodeLab3

//All files including testbench

# CompArchLab3RegFile

Full Register File With Test Bench

module Lab3RegisterFile(Q, D, L, R, clock);

	parameter N = 32;
  
	output reg [N-1:0]Q; // registered output
  
	input [N-1:0]D; // data input
  
	input L;
  
	input R;
  
	input clock;
  
	always @(posedge clock or posedge R) begin
  
	if(R)
  
		Q <= 0;	
    
	else if (L)
  
		Q<= D;
    
	else
  
		Q<=Q;		
    
	end
  
	endmodule 

  
  module Mux(DataOut,Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7,Data8,Data9,Data10,Data11,Data12,Data13,Data14,Data15,Data16,Data17,Data18,Data19,Data20,Data21,Data22,Data23,Data24,Data25,Data26,Data27,Data28,Data29,Data30,Data31,select);

output reg [31:0] DataOut; 

input [31:0] Data0,Data1,Data2,Data3,Data4,Data5,Data6,Data7,Data8,Data9,Data10,Data11,Data12,Data13,Data14,Data15,Data16,Data17,Data18,Data19,Data20,Data21,Data22,Data23,Data24,Data25,Data26,Data27,Data28,Data29,Data30,Data31;

input [4:0]select;


always @(*) begin

case (select)
5'd0: DataOut <= Data0;

5'd1: DataOut <= Data1; 

5'd2: DataOut <= Data2;

5'd3: DataOut <= Data3;

5'd4: DataOut <= Data4;

5'd5: DataOut <= Data5;

5'd6: DataOut <= Data6;

5'd7: DataOut <= Data7;

5'd8: DataOut <= Data8;

5'd9: DataOut <= Data9;

5'd10: DataOut <= Data10;

5'd11: DataOut <= Data11;

5'd12: DataOut <= Data12;

5'd13: DataOut <= Data13;

5'd14: DataOut <= Data14;

5'd15: DataOut <= Data15;

5'd16: DataOut <= Data16;

5'd17: DataOut <= Data17;

5'd18: DataOut <= Data18;

5'd19: DataOut <= Data19;

5'd20: DataOut <= Data20;

5'd21: DataOut <= Data21;

5'd22: DataOut <= Data22;

5'd23: DataOut <= Data23;

5'd24: DataOut <= Data24;

5'd25: DataOut <= Data25;

5'd26: DataOut <= Data26;

5'd27: DataOut <= Data27;

5'd28: DataOut <= Data28;

5'd29: DataOut <= Data29;

5'd30: DataOut <= Data30;

5'd31: DataOut <= Data31;

endcase

end

endmodule 

module Decoder5to32(m, s, en);

	input [4:0]s;
  
	input en;
  
	output [31:0]m;
	
	assign m[0] = ~s[4] & ~s[3] & ~s[2] & ~s[1] & ~s[0] & en; //00000
  
	assign m[1] = ~s[4] & ~s[3] & ~s[2] & ~s[1] & s[0] & en; // 00001
  
	assign m[2] = ~s[4] & ~s[3] & ~s[2] & s[1] & ~s[0] & en; // 00010
  
	assign m[3] = ~s[4] & ~s[3] & ~s[2] & s[1] & s[0] & en;  // 00011
  
	assign m[4] = ~s[4] & ~s[3] & s[2] & ~s[1] & ~s[0] & en; // 00100
  
	assign m[5] = ~s[4] & ~s[3] & s[2] & ~s[1] & s[0] & en;  // 00101
  
	assign m[6] = ~s[4] & ~s[3] & s[2] & s[1] & ~s[0] & en;  // 00110
  
	assign m[7] = ~s[4] & ~s[3] & s[2] & s[1] & s[0] & en;   // 00111
  
	assign m[8] = ~s[4] & s[3] & ~s[2] & ~s[1] & ~s[0] & en; // 01000
  
	assign m[9] = ~s[4] & s[3] & ~s[2] & ~s[1] & s[0] & en;  // 01001
  
	assign m[10] = ~s[4] & s[3] & ~s[2] & s[1] & ~s[0] & en; // 01010
  
	assign m[11] = ~s[4] & s[3] & ~s[2] & s[1] & s[0] & en;  // 01011
  
	assign m[12] = ~s[4] & s[3] & s[2] & ~s[1] & ~s[0] & en; // 01100
  
	assign m[13] = ~s[4] & s[3] & s[2] & ~s[1] & s[0] & en;  // 01101
  
	assign m[14] = ~s[4] & s[3] & s[2] & s[1] & ~s[0] & en;  // 01110
  
	assign m[15] = ~s[4] & s[3] & s[2] & s[1] & s[0] & en;   // 01111
  
	assign m[16] = s[4] & ~s[3] & ~s[2] & ~s[1] & ~s[0] & en;// 10000
  
	assign m[17] = s[4] & ~s[3] & ~s[2] & ~s[1] & s[0] & en; // 10001
  
	assign m[18] = s[4] & ~s[3] & ~s[2] & s[1] & ~s[0] & en; // 10010
  
	assign m[19] = s[4] & ~s[3] & ~s[2] & s[1] & s[0] & en;  // 10011
  
	assign m[20] = s[4] & ~s[3] & s[2] & ~s[1] & ~s[0] & en; // 10100
  
	assign m[21] = s[4] & ~s[3] & s[2] & ~s[1] & ~s[0] & en; // 10100
  
	assign m[22] = s[4] & ~s[3] & s[2] & ~s[1] & s[0] & en; //  10101
  
	assign m[23] = s[4] & ~s[3] & s[2] & s[1] & ~s[0] & en; //  10110
  
	assign m[24] = s[4] & ~s[3] & s[2] & s[1] & s[0] & en; //   10111
  
	assign m[25] = s[4] & s[3] & ~s[2] & ~s[1] & ~s[0] & en; // 11000
  
	assign m[26] = s[4] & s[3] & ~s[2] & ~s[1] & s[0] & en; //  11001
  
	assign m[27] = s[4] & s[3] & ~s[2] & s[1] & ~s[0] & en; //  11010
  
	assign m[28] = s[4] & ~s[3] & ~s[2] & s[1] & s[0] & en; //  11011
  
	assign m[29] = s[4] & s[3] & s[2] & ~s[1] & ~s[0] & en; //  11100
  
	assign m[30] = s[4] & s[3] & s[2] & s[1] & ~s[0] & en; //   11110
  
	assign m[31] = s[4] & s[3] & s[2] & s[1] & s[0] & en; //    11111
	
endmodule

module Test(D_C, D_A, clk, rst, en, D_B, W_rd, rd_adr_A, rd_adr_B);

parameter N=32;

input [N-1:0] D_C;

input W_rd;

input clk, rst, en;

input [4:0]rd_adr_A;

input [4:0]rd_adr_B;

output [N-1:0]D_A, D_B;

wire [N-1:0] load;

wire [N-1:0] r00,r01,r02,r03,r04,r05,r06,r07,r08,r09,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r27,r28,r29,r30,r31;

Decoder5to32 D_Decoder(load, W_rd, en);

//32 register
Lab3RegisterFile Reg00(r00, D_C, load[0], rst, clk); 

Lab3RegisterFile Reg01(r01, D_C, load[1], rst, clk); 

Lab3RegisterFile Reg02(r02, D_C, load[2], rst, clk); 

Lab3RegisterFile Reg03(r03, D_C, load[3], rst, clk); 

Lab3RegisterFile Reg04(r04, D_C, load[4], rst, clk); 

Lab3RegisterFile Reg05(r05, D_C, load[5], rst, clk); 

Lab3RegisterFile Reg06(r06, D_C, load[6], rst, clk); 

Lab3RegisterFile Reg07(r07, D_C, load[7], rst, clk); 

Lab3RegisterFile Reg08(r08, D_C, load[8], rst, clk); 

Lab3RegisterFile Reg09(r09, D_C, load[9], rst, clk); 

Lab3RegisterFile Reg10(r10, D_C, load[10], rst, clk); 

Lab3RegisterFile Reg11(r11, D_C, load[11], rst, clk); 

Lab3RegisterFile Reg12(r12, D_C, load[12], rst, clk); 

Lab3RegisterFile Reg13(r13, D_C, load[13], rst, clk); 

Lab3RegisterFile Reg14(r14, D_C, load[14], rst, clk); 

Lab3RegisterFile Reg15(r15, D_C, load[15], rst, clk); 

Lab3RegisterFile Reg16(r16, D_C, load[16], rst, clk); 

Lab3RegisterFile Reg17(r17, D_C, load[17], rst, clk); 

Lab3RegisterFile Reg18(r18, D_C, load[18], rst, clk); 

Lab3RegisterFile Reg19(r19, D_C, load[19], rst, clk); 

Lab3RegisterFile Reg20(r20, D_C, load[20], rst, clk); 

Lab3RegisterFile Reg21(r21, D_C, load[21], rst, clk); 

Lab3RegisterFile Reg22(r22, D_C, load[22], rst, clk); 

Lab3RegisterFile Reg23(r23, D_C, load[23], rst, clk); 

Lab3RegisterFile Reg24(r24, D_C, load[24], rst, clk); 

Lab3RegisterFile Reg25(r25, D_C, load[25], rst, clk); 

Lab3RegisterFile Reg26(r26, D_C, load[26], rst, clk); 

Lab3RegisterFile Reg27(r27, D_C, load[27], rst, clk); 

Lab3RegisterFile Reg28(r28, D_C, load[28], rst, clk); 

Lab3RegisterFile Reg29(r29, D_C, load[29], rst, clk); 

Lab3RegisterFile Reg30(r30, D_C, load[30], rst, clk); 

Lab3RegisterFile Reg31(r31, D_C, load[31], rst, clk); 

//mux

Mux(D_A,r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r7,r28,r29,r30,r31,rd_adr_A);

Mux(D_B,r0,r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21,r22,r23,r24,r25,r26,r7,r28,r29,r30,r31,rd_adr_B);

endmodule 

`timescale 10ns/10ns

module TestBench();

reg[31:0] D_C;

reg[4:0] rd_adr_A;

reg[4:0] rd_adr_B;

reg clk, rst;

reg W_rd=1;

reg en=1;

reg [4:0] D;

wire [31:0] D_A,D_B;

Test dut(D_C, D_A, clk, rst, en, D_B, W_rd, rd_adr_A, rd_adr_B);

initial begin

clk = 1b'0;

rst = 1b'1;

D = 5'd0;

rd_adr_A = 5'd0;

rd_adr_B = 5'd0;

#10

rst = 1'b0;

#10

rd_adr_A = 5'd3;

rd_adr_B = 5'd3;

#10

$stop;

end

 always begin 
 
 #5
 
 clk = ~clk;
 
 end

endmodule 
