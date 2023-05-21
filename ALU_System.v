 `timescale 1ns / 1ps
`include "ALU.v"
`include "Memory.v"
`include "IR.v"
`include "ARF.v"
`include "RF.v"
`include "MUX_4bit.v"
`include "MUX_2bit.v"


module ALU_System(
    input [2:0] RF_OutASel, // pass
    input [2:0] RF_OutBSel, // pass
    input [1:0] RF_FunSel,
    input [3:0] RF_RSel,
    input [3:0] RF_TSel,
    input [3:0] ALU_FunSel,
    input [1:0] ARF_OutCSel, 
    input [1:0] ARF_OutDSel, 
    input [1:0] ARF_FunSel,
    input [3:0] ARF_RegSel, // pass
    input IR_LH,
    input IR_Enable,
    input [1:0] IR_Funsel,
    input Mem_WR,
    input Mem_CS,
    input [1:0] MuxASel,
    input [1:0] MuxBSel,
    input MuxCSel,
    input Clock,
    output [7:0] AOut,
    output [7:0] BOut,
    output [7:0] ALUOut,
    output [3:0] ALUOutFlag,
    output [7:0] ARF_COut,     // pass
    output [7:0] ARF_DOut,
    output [7:0] Address,
    output [7:0] MemoryOut,
    output [15:0] IROut,
    output [7:0] MuxAOut,
    output [7:0] MuxBOut,
    output [7:0] MuxCOut  
    );
    
    ALU alu1(
    .A(MuxC.out),
    .B(rf1.Output1),
    .FunSel(ALU_FunSel)
    );
    
    Memory mem1(
    .address(arf1.OutB),
    .data(alu1.OutALU),
    .wr(Mem_WR),
    .cs(Mem_CS),
    .clock(Clock)
    );
    
    IR ir1(
    .E(IR_Enable),
    .FunSel(IR_Funsel),
    .Input(mem1.o),
    .LH(IR_LH)
    );
    
    ARF arf1(
    .Input(MuxB.out),
    .OASel(ARF_OutCSel),
    .OBSel(ARF_OutDSel),
    .FunSel(ARF_FunSel),
    .RSel(ARF_RegSel),
    .clock(Clock)
    );
    
    RF rf1(
    .Input(MuxA.out),
    .O1Sel(RF_OutASel),// bit farkli
    .O2Sel(RF_OutBSel),// bit fakli
    .FunSel(RF_FunSel),
    .RSel(RF_RSel),
    .TSel(RF_TSel)
    );
    
    
    MUX_4bit MuxA(
    .input_1(alu1.OutALU), // ALUOut
    .input_2(mem1.o),
    .input_3(ir1.IROut[7:0]),
    .input_4(arf1.OutA),
    .select(MuxASel),
    .out(MuxAOut)
    );
    MUX_4bit MuxB(
    .input_1(alu1.OutALU), // ALUOut
    .input_2(mem1.o),
    .input_3(ir1.IROut[7:0]),
    .input_4(arf1.OutA),
    .select(MuxBSel),
    .out(MuxBOut)
    );
    
    MUX_2bit MuxC(
    .input_1(rf1.Output1), // ALUOut
    .input_2(arf1.OutA),
    .select(MuxCSel),
    .out(MuxCOut)
    );
    
    assign AOut = rf1.Output1; // MuxC output olarak da degisebilir
    assign BOut = rf1.Output2;
    assign ALUOut = alu1.OutALU;
    assign ALUOutFlag = alu1.OutFlag;
    assign ARF_COut = arf1.OutA; // pass
    assign Address = arf1.OutB;
    assign MemoryOut = mem1.o;
    assign IROut = ir1.IROut[15:0];
    assign MuxAOut = MuxA.out;
    assign MuxBOut = MuxB.out;
    assign MuxCOut = MuxC.out;
    
    
endmodule