`timescale 1ns / 1ps
`include "ALU_System.v"
`include "ControlUnit.v"

module Computer_Test (
    
);
    reg clock;
    initial begin

        // Initialize the clock signal
        clock = 0;
        // Run the simulation for 5 clock cycles
        repeat (300) begin
            #10 clock = ~clock;
        end
        // Finish the simulation
        $finish;
    end
    ALU_System alu_system1(
        .Clock(clock),
        .RF_OutASel(control1.RF_OutASel),
        .RF_OutBSel(control1.RF_OutBSel),
        .RF_FunSel(control1.RF_FunSel),
        .RF_RSel(control1.RF_RSel),
        .RF_TSel(control1.RF_TSel),
        .ALU_FunSel(control1.ALU_FunSel),
        .ARF_OutCSel(control1.ARF_OutCSel),
        .ARF_OutDSel(control1.ARF_OutDSel),
        .ARF_FunSel(control1.ARF_FunSel),
        .ARF_RegSel(control1.ARF_RegSel),
        .IR_LH(control1.IR_LH),
        .IR_Enable(control1.IR_Enable),
        .IR_Funsel(control1.IR_Funsel),
        .Mem_WR(control1.Mem_WR),
        .Mem_CS(control1.Mem_CS),
        .MuxASel(control1.MuxASel),
        .MuxBSel(control1.MuxBSel),
        .MuxCSel(control1.MuxCSel)
    );

    ControlUnit control1(
        .clock(clock),
        .AOut(alu_system1.AOut),
        .BOut(alu_system1.BOut),
        .ALUOut(alu_system1.ALUOut),
        .ALUOutFlag(alu_system1.ALUOutFlag),
        .ARF_COut(alu_system1.ARF_COut),
        .Address(alu_system1.Address),
        .MemoryOut(alu_system1.MemoryOut),
        .IROut(alu_system1.IROut),
        .MuxAOut(alu_system1.MuxAOut),
        .MuxBOut(alu_system1.MuxBOut),
        .MuxCOut(alu_system1.MuxCOut)
    );


    
endmodule