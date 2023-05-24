// Mehmet Umut Gökdağ 150200085
// Fırat Kızılboğa    150200115
// Nurselen Akın      150200087
`timescale 1ns / 1ps

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
    .B(rf1.Output2),
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
    .select(MuxASel)
    );
    MUX_4bit MuxB(
    .input_1(alu1.OutALU), // ALUOut
    .input_2(mem1.o),
    .input_3(ir1.IROut[7:0]),
    .input_4(arf1.OutA),
    .select(MuxBSel)
    );
    
    MUX_2bit MuxC(
    .input_1(rf1.Output1), // ALUOut
    .input_2(arf1.OutA),
    .select(MuxCSel)
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

module ALU(
    A,B,FunSel,OutALU,OutFlag,RESET,Clock
    );
    input wire [7:0] A;
    input wire [7:0] B;
    input wire RESET;
    input wire Clock;
    input wire [3:0] FunSel;
    output wire [7:0] OutALU;
    output wire [3:0] OutFlag;

    reg [8:0] result = 0;

    reg ZERO = 1'bX;
    reg SET_ZERO;
    reg CARRY = 1'bX;
    reg SET_CARRY;
    reg NEGATIVE = 1'bX;
    reg SET_NEGATIVE;
    reg OVERFLOW = 1'bX;
    reg SET_OVERFLOW;

    //OVERFLOW SHOULD BE CALLED BEFORE CARRY

    always @(FunSel) begin
        #4;
        $display("ALU OPERATION A = %x, B = %x, Result = %x, FunSel = %x", A, B, result[7:0], FunSel);
    end

    always @(A,B,FunSel,Clock) begin
        case (FunSel) 
            4'b0000: begin // A 
                result <= A; // A 
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b0001: begin // B
                result <= B; // B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b0010: begin // not A
                result <= ~A; // not A
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b0011: begin // not B
                result <= ~B; // not B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b0100: begin // A + B
                result <= A + B ; // A + B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
                SET_OVERFLOW <= 1'b1;
                SET_CARRY <= 1'b1;
            end
            4'b0101: begin // A - B
                result <= A - B ;#1;
                SET_ZERO <= 1'b1; 
                SET_NEGATIVE <= 1'b1;
                SET_OVERFLOW <= 1'b1;
                SET_CARRY <= 1'b1;
            end
            4'b0110: begin // Compare A , B
                if (A > B) begin
                    result <= A; // A is greater, return A
                end else begin
                    result <= 8'd0; // A is not greater, return 0
                end
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
                SET_OVERFLOW <= 1'b1;
                SET_CARRY <= 1'b1;
            end
            4'b0111: begin // A and B
                result <= A & B; // A and B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b1000: begin // A or B
                result <= A | B; // A or B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b1001: begin // A NAND B
                result <= ~(A & B); // A NAND B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b1010: begin // A xor B
                result <= A ^ B; // A xor B
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
            4'b1011: begin // Logical shift left A and insert 0
                result <= {A, 1'b0}; // Logical shift left A and insert 0
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
                CARRY <= A[7];
            end
            4'b1100: begin // Logical shift right A and insert 0
                result <= {2'b00, A[7:1]}; // Logical shift right A and insert 0
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
                CARRY <= A[0];
            end
            4'b1101: begin // arithmetic shift left A and insert 0
                result <= {A, 1'b0}; // arithmetic shift left A and insert 0
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
                SET_OVERFLOW <= 1'b1;
            end
            4'b1110: begin // arithmetic shift right A and insert 0
                result <= {1'b0, A[7] ,A[7:1]}; // arithmetic shift right A and insert 0
                SET_ZERO <= 1'b1;
            end
            4'b1111: begin //CSL 
                result <= {1'b0,CARRY, A[7:1]}; //CSL
                CARRY <= A[0];
                SET_ZERO <= 1'b1;
                SET_NEGATIVE <= 1'b1;
            end
        endcase
    end
    //clock rasing edge
    always @(posedge SET_NEGATIVE) begin
        NEGATIVE <= OutALU[7];
        SET_NEGATIVE <= 1'b0;
    end

    always @(posedge SET_ZERO) begin
            ZERO <= ~(|OutALU[7:0]);
        SET_ZERO <= 1'b0;
    end

    always @(posedge SET_CARRY) begin
        CARRY <= result[8];
        SET_CARRY <= 1'b0;
    end

    always @(posedge SET_OVERFLOW) begin
        if (A[7] == B[7] && result[7] != A[7]) begin
            OVERFLOW <= 1'b1;
        end else begin
            OVERFLOW <= 1'b0;
        end
        SET_OVERFLOW <= 1'b0;
    end

    always @(posedge RESET) begin
        $display("RESET");
        ZERO <= 1'b0;
        CARRY <= 1'b0;
        NEGATIVE <= 1'b0;
        OVERFLOW <= 1'b0;
        result <= 9'b0;
    end

    assign OutFlag = {ZERO, CARRY, NEGATIVE, OVERFLOW};
    assign OutALU = result[7:0];
endmodule

module ARF(
     input [7:0] Input, 
     input [1:0] OASel,
     input [1:0] OBSel,
     input [1:0] FunSel,
     input [3:0] RSel,
     input clock,
     output reg [7:0] OutA,
     output reg [7:0] OutB
     );
     reg [7:0] PC = 0;
     reg [7:0] AR = 0;
     reg [7:0] SP = 0;
     reg [7:0] PCpast = 0;

     reg SET_AR, SET_SP, SET_PC, SET_PCPAST;


     always @(RSel,OASel,OBSel,FunSel) begin
          case (RSel)
               4'b0000: begin 
               end
               4'b0001: begin
               SET_PCPAST <= 1'b1;
               end
               4'b0010: begin
               SET_SP <= 1'b1;
               end
               4'b0011: begin
               SET_SP <= 1'b1;
               SET_PCPAST <= 1'b1;
               end
               4'b0100: begin
               SET_AR <= 1'b1;
               end
               4'b0101: begin
               SET_AR <= 1'b1;
               SET_PCPAST <= 1'b1;
               end
               4'b0110: begin
               SET_AR <= 1'b1;
               SET_SP <= 1'b1;
               end
               4'b0111: begin
               SET_AR <= 1'b1;
               SET_SP <= 1'b1;
               SET_PCPAST <= 1'b1;
               end       
               4'b1000: begin 
               SET_PC <= 1'b1;
               end
               4'b1001: begin
               SET_PC <= 1'b1;
               SET_PCPAST <= 1'b1;
               end
               4'b1010: begin
               SET_PC <= 1'b1;
               SET_SP <= 1'b1;
               end
               4'b1011: begin
               SET_PC <= 1'b1;
               SET_SP <= 1'b1;
               SET_PCPAST <= 1'b1;
               end
               4'b1100: begin
               SET_PC <= 1'b1;
               SET_AR <= 1'b1;
               end
               4'b1101: begin
               SET_PC <= 1'b1;
               SET_AR <= 1'b1;
               SET_PCPAST <= 1'b1;
               end
               4'b1110: begin
               SET_PC <= 1'b1;
               SET_AR <= 1'b1;
               SET_SP <= 1'b1;
               end
               4'b1111: begin
               SET_PC <= 1'b1;
               SET_AR <= 1'b1;
               SET_SP <= 1'b1;
               SET_PCPAST <= 1'b1;
               end                                                                                                                                                        
        endcase
    end
     always @( posedge SET_AR) begin
        
          if (FunSel == 2'b00) begin
               AR = 8'd0;
          end
          else if (FunSel == 2'b01) begin
               AR = Input;
          end
          else if (FunSel == 2'b10) begin
               AR = AR - 8'd1;
          end     
          else if (FunSel == 2'b11) begin
               AR = AR + 8'd1;
          end  
          SET_AR  <= 1'b0;
     end
     always @( posedge SET_SP) begin
          if (FunSel == 2'b00) begin
               SP = 8'd0;
          end
          else if (FunSel == 2'b01) begin
               SP = Input;
          end
          else if (FunSel == 2'b10) begin
               SP = SP - 8'd1;
          end     
          else if (FunSel == 2'b11) begin
               SP = SP + 8'd1;
          end 
          SET_SP  <= 1'b0; 
     end

     always @( posedge SET_PCPAST) begin
          if (FunSel == 2'b00) begin
               PCpast = 8'd0;
          end
          else if (FunSel == 2'b01) begin
               PCpast = Input;
          end
          else if (FunSel == 2'b10) begin
               PCpast = PCpast - 8'd1;
          end     
          else if (FunSel == 2'b11) begin
               PCpast = PCpast + 8'd1;
          end  
          SET_PCPAST  <= 1'b0; 
     end
     always @(posedge SET_PC) begin
          if (FunSel == 2'b00) begin
               PC = 8'd0;
          end
          else if (FunSel == 2'b01) begin
               PC = Input;
          end
          else if (FunSel == 2'b10) begin
               PC = PC - 8'd1;
          end     
          else if (FunSel == 2'b11) begin
               PC = PC + 8'd1;
          end  
          SET_PC  <= 1'b0;
     end
    
     always @ (OASel or PC or AR or SP or PCpast) begin
          case (OASel)
               2'b00: OutA = AR;
               2'b01: OutA = SP;
               2'b10: OutA = PCpast;
               2'b11: OutA = PC;
          endcase
        
    end
     always @ (OBSel or PC or AR or SP or PCpast) begin
          case (OBSel)
               2'b00: OutB = AR;
               2'b01: OutB = SP;
               2'b10: OutB = PCpast;
               2'b11: OutB = PC;
          endcase
     end

     always @(PC, AR, SP, PCpast) begin
          $display("Change in ARF:\nPC = %d, AR = %d, SP = %d, PCpast = %d", PC, AR, SP, PCpast);
     end
endmodule

module CPUSystem(
    input wire Clock,
    input wire Reset,
    input wire [7:0] T
    );
    wire real_clock;
    assign real_clock = !Clock;
    ALU_System alu_system1(
        .Clock(real_clock),
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
        .clock(real_clock),
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

module ControlUnit(
    input clock,
    input [7:0] AOut,
    input [7:0] BOut,
    input [7:0] ALUOut,
    input [3:0] ALUOutFlag,
    input [7:0] ARF_COut,     // pass
    input [7:0] Address,
    input [7:0] MemoryOut,
    input [15:0] IROut,
    input [7:0] MuxAOut,
    input [7:0] MuxBOut,
    input [7:0] MuxCOut,
    output [2:0] RF_OutASel, 
    output [2:0] RF_OutBSel, // pass
    output [1:0] RF_FunSel,
    output [3:0] RF_RSel,
    output [3:0] RF_TSel,
    output [3:0] ALU_FunSel,
    output [1:0] ARF_OutCSel, 
    output [1:0] ARF_OutDSel, 
    output [1:0] ARF_FunSel,
    output [3:0] ARF_RegSel, // pass
    output IR_LH,
    output IR_Enable,
    output [1:0] IR_Funsel,
    output Mem_WR,
    output Mem_CS,
    output [1:0] MuxASel,
    output [1:0] MuxBSel,
    output MuxCSel
    );

    reg [2:0] reg_RF_OutASel;
    reg [2:0] reg_RF_OutBSel;
    reg [1:0] reg_RF_FunSel;
    reg [3:0] reg_RF_RSel;
    reg [3:0] reg_RF_TSel;
    reg [3:0] reg_ALU_FunSel;
    reg [1:0] reg_ARF_OutCSel;
    reg [1:0] reg_ARF_OutDSel;
    reg [1:0] reg_ARF_FunSel;
    reg [3:0] reg_ARF_RegSel;
    reg reg_IR_LH;
    reg reg_IR_Enable;
    reg [1:0] reg_IR_Funsel;
    reg reg_Mem_WR;
    reg reg_Mem_CS;
    reg [1:0] reg_MuxASel;
    reg [1:0] reg_MuxBSel;
    reg reg_MuxCSel;

    assign RF_OutASel = reg_RF_OutASel;
    assign RF_OutBSel = reg_RF_OutBSel;
    assign RF_FunSel = reg_RF_FunSel;
    assign RF_RSel = reg_RF_RSel;
    assign RF_TSel = reg_RF_TSel;
    assign ALU_FunSel = reg_ALU_FunSel;
    assign ARF_OutCSel = reg_ARF_OutCSel;
    assign ARF_OutDSel = reg_ARF_OutDSel;
    assign ARF_FunSel = reg_ARF_FunSel;
    assign ARF_RegSel = reg_ARF_RegSel;
    assign IR_LH = reg_IR_LH;
    assign IR_Enable = reg_IR_Enable;
    assign IR_Funsel = reg_IR_Funsel;
    assign Mem_WR = reg_Mem_WR;
    assign Mem_CS = reg_Mem_CS;
    assign MuxASel = reg_MuxASel;
    assign MuxBSel = reg_MuxBSel;
    assign MuxCSel = reg_MuxCSel;

    reg I;
    wire [3:0] OPCODE; 
    wire [3:0] SREG1;
    wire [3:0] SREG2;
    wire [3:0] DSTREG;
    reg [3:0] SREGA;
    reg [3:0] SREGB; 

    assign DSTREG = IROut[11:8];
    assign SREG1 = IROut[7:4];
    assign SREG2 = IROut[3:0];

    wire ADDRESSING_MODE;
    wire [1:0] RSEL;
    wire [7:0] ADDRESS;

    assign ADDRESS = IROut[7:0];
    assign RSEL = IROut[9:8];
    assign ADDRESSING_MODE = IROut[10];

    assign OPCODE = IROut[15:12];

    reg [7:0] VALUE;

    reg [1:0] seq_counter = 0;
    assign seq = seq_counter;
    always @(posedge clock) begin
        seq_counter <= seq_counter + 1;
        $display("*******************");
        $display("SEQ = %d", seq_counter);
        $display("Address = %h", Address);
        $display("IRFunsel = %h", IR_Funsel);
        $display("IR_LH = %h", IR_LH);
        $display("IR_Enable = %h", IR_Enable);
        $display("MemoryOut = %h", MemoryOut);
        $display("IROut = %h", IROut);
        $display("ALUOut = %h", ALUOut);
        $display("ALUOutFlag = %b", ALUOutFlag);
        $display("MuxAOut = %h", MuxAOut);
        $display("MuxASel = %h", MuxASel);
        $display("MuxBOut = %h", MuxBOut);
        $display("MuxBSel = %h", MuxBSel);
        $display("MuxCOut = %h", MuxCOut);
        $display("MuxCSel = %h", MuxCSel);
        $display("ARF_FunSel = %h", ARF_FunSel);
        $display("ARF_RegSel = %h", ARF_RegSel);
        $display("ARF_OutCSel = %h", ARF_OutCSel);
        $display("ARF_OutDSel = %h", ARF_OutDSel);
        $display("RF_FunSel = %h", RF_FunSel);
        $display("RF_RSel = %h", RF_RSel);
        $display("RF_OutASel = %h", RF_OutASel);
        $display("RF_OutBSel = %h", RF_OutBSel);
        $display("*******************");
    end

    always @(seq_counter) begin
            // Set default values for all control signals   
        reg_ARF_FunSel = 2'bX;
        reg_ARF_RegSel = 4'bX;
        reg_IR_Enable = 1'b0;
        reg_Mem_WR = 1'b0;
        reg_Mem_CS = 1'b0;
        reg_RF_FunSel = 2'bX;
        reg_RF_RSel = 4'bX;
        reg_IR_Funsel = 2'bX;
        
        SREGA = (SREG1 > SREG2) ? SREG1 : SREG2;
        SREGB = (SREG1 > SREG2) ? SREG2 : SREG1;

        if (seq_counter == 2'b00) begin // T0: AR <- PC // pc = 0
            $display("T0 Fetching low");
            reg_IR_Enable = 1; // read before increment problem
            reg_IR_LH = 0;
            reg_ARF_OutDSel = 2'b11; // PC
            reg_IR_Funsel = 2'b01;
            #5;
            reg_ARF_FunSel = 2'b11; // INC
            reg_ARF_RegSel = 4'b1000; // PC
            //print ar and pc
        end

        else if (seq_counter == 2'b01) begin // T1: AR <- PC  // pc = 1
            $display("T1 Fetching high");
            reg_IR_Enable = 1;
            reg_IR_LH = 1;
            reg_IR_Funsel = 2'b01;
            #0.5;
            reg_ARF_FunSel = 2'b11; // INC
            reg_ARF_RegSel = 4'b1000; // PC
        end

        else begin 
            case(OPCODE)
                4'b0000 : begin //AND
                    if (seq_counter == 2'b10) begin // T2: RF <- RF & RF
                        update_SREGA_flag = 1;
                        update_SREGB_flag = 1;
                    end
                    if (seq_counter == 2'b11) begin // T3: RF <- RF & RF
                        reg_ALU_FunSel = 4'b0111;
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0001 : begin //OR
                    if (seq_counter == 2'b10) begin // T2: RF <- RF | RF
                        update_SREGA_flag = 1;
                        update_SREGB_flag = 1;
                        reg_ALU_FunSel = 4'b1000;
                        //t3
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0010 : begin //NOT
                    if (seq_counter == 2'b10) begin // T2: RF <- RF'
                        SREGA = SREG1;
                        update_SREGA_flag = 1;
                        reg_ALU_FunSel = 4'b0010;
                        //t3
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0011 : begin //ADD
                    if (seq_counter == 2'b10) begin // T2: RF <- RF + RF
                        update_SREGA_flag = 1;
                        update_SREGB_flag = 1;
                        reg_ALU_FunSel = 4'b0100;
                        //t3
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0100 : begin //SUB
                    if (seq_counter == 2'b10) begin // T2: RF <- RF - RF
                        SREGA = SREG1;
                        SREGB = SREG2;
                        #1;
                        update_SREGA_flag = 1;
                        update_SREGB_flag = 1;
                        #1;
                        reg_ALU_FunSel = 4'b0101;
                        //t3
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0101 : begin //LSR
                    if (seq_counter == 2'b10) begin // T2: RF <- RF + 1
                        SREGA = SREG1;
                        update_SREGA_flag = 1;
                        reg_ALU_FunSel = 4'b1100;
                        //t3
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0110 : begin //LSL
                    if (seq_counter == 2'b10) begin // T2: RF <- RF - 1
                        SREGA = SREG1;
                        update_SREGA_flag = 1;
                        reg_ALU_FunSel = 4'b1011;
                        //t3
                        update_DSTREG_flag = 1;
                    end
                end
                4'b0111 : begin //INC
                    if (seq_counter == 2'b10) begin
                        SREGA = SREG1;
                        reg_ALU_FunSel = 4'b0000;
                        update_SREGA_flag = 1;
                        update_DSTREG_flag = 1;
                    end
                    if (seq_counter == 2'b11) begin // T2: RF <- RF + 1
                        update_SREG1_flag = 1;
                        if(SREG1 > 4'd3) begin
                            reg_ARF_FunSel = 2'b11; // INC
                        end
                        else begin
                            reg_RF_FunSel = 2'b11; // INC
                        end
                    end
                    #1;
                end
                4'b1000 : begin //DEC
                    if (seq_counter == 2'b10) begin
                        SREGA = SREG1;
                        reg_ALU_FunSel = 4'b0000;
                        update_SREGA_flag = 1;
                        update_DSTREG_flag = 1;
                    end
                    if (seq_counter == 2'b11) begin // T2: RF <- RF + 1
                        update_SREG1_flag = 1;
                        if(SREG1 > 4'd3) begin
                            reg_ARF_FunSel = 2'b10; // INC
                        end
                        else begin
                            reg_RF_FunSel = 2'b10; // INC
                        end
                    end
                    #1;
                end
                4'b1001 : begin //BRA
                    if (seq_counter == 2'b10) begin //BRA
                        reg_MuxBSel = 2'b10;
                        reg_ARF_RegSel = 4'b1000; //PC
                        reg_ARF_FunSel = 2'b01; // LOAD
                        #2.5;
                        reg_ARF_RegSel = 4'b1000; //PC
                        reg_ARF_FunSel = 2'b10; // DEC
                        //seq_counter = 4'hff;
                    end
                end
                4'b1010 : begin //BNE
                    if (seq_counter == 2'b10) begin //BNE
                        if(ALUOutFlag == 4'd8) begin
                            $display("BNE alu 1");
                            $display("ALUOUT = %d", ALUOut);
                            $display("ALUFLAG = %d", ALUOutFlag);
                        end
                        else begin 
                            $display("BNE alu 0");
                            $display("ALUOUT = %d", ALUOut);
                            $display("ALUFLAG = %d", ALUOutFlag);
                            reg_MuxBSel = 2'b10;
                            reg_ARF_RegSel = 4'b1000; //PC
                            reg_ARF_FunSel = 2'b01; // LOAD
                            #2.5;
                            reg_ARF_RegSel = 4'b1000; //PC
                            reg_ARF_FunSel = 2'b10; // DEC
                            //seq_counter = 4'hff;
                        end

                        
                    end

                end
                4'b1011 : begin //MOV
                    if (seq_counter == 2'b10) begin
                        update_SREGA_flag = 1;
                        reg_ALU_FunSel = 4'b0000;
                    end
                    else if (seq_counter == 2'b11) begin
                        update_DSTREG_flag = 1;
                        #2.5;
                        //seq_counter = 4'hff;
                    end
                end
                4'b1100 : begin //LD
                    if (seq_counter == 2'b10) begin //
                        if (ADDRESSING_MODE) begin //DIRECT 
                            reg_ARF_OutDSel <= 2'b00; // AR
                            //reg_ARF_FunSel <= 2'b01; // LOAD 
                            //reg_ARF_RegSel <= 4'b0100; 
                        end
                    end
                    if (seq_counter == 2'b11) begin //BRA
                        reg_MuxASel = (ADDRESSING_MODE) ? 2'b01 : 2'b10;
                        reg_RF_FunSel = 2'b01; // LOAD
                        case (RSEL)
                            2'b00 : begin
                                reg_RF_RSel = 4'b1000; //R1
                            end
                            2'b01 : begin
                                reg_RF_RSel = 4'b0100; //R2
                            end
                            2'b10 : begin
                                reg_RF_RSel = 4'b0010; //R3
                            end
                            2'b11 : begin
                                reg_RF_RSel = 4'b0001; //R4
                            end
                        endcase
                        #2.5;
                        //seq_counter <= 4'hff;
                    end
                end
                4'b1101 : begin //ST
                    if (seq_counter == 2'b10) begin // AR <- IR(7-0)
                        reg_ARF_OutDSel <= 2'b00; // AR
                        #2.5;
                        reg_MuxBSel <= 2'b11; // MEMORY OUT
                        reg_ARF_FunSel <= 2'b01; // LOAD 
                        reg_ARF_RegSel <= 4'b0100; 
                        #2;
                    end
                    if (seq_counter == 2'b11) begin //
                        case (RSEL)
                            2'b00 : begin
                                reg_RF_OutBSel = 3'b100; //R1
                            end
                            2'b01 : begin
                                reg_RF_OutBSel = 3'b101; //R2
                            end
                            2'b10 : begin
                                reg_RF_OutBSel = 3'b110; //R3
                            end
                            2'b11 : begin
                                reg_RF_OutBSel = 3'b111; //R4
                            end
                        endcase
                    reg_ALU_FunSel = 4'b0001; //Pass B
                    reg_Mem_WR = 1; // write
                    #2.5;

                    end
                end
                4'b1110 : begin //PULL               
                    if (seq_counter == 2'b10) begin //
                        reg_ARF_OutDSel = 2'b01;
                        reg_MuxASel = 2'b01; // SP
                        case (RSEL)
                            2'b00 : begin
                                reg_RF_RSel = 4'b1000; //R1
                            end
                            2'b01 : begin
                                reg_RF_RSel = 4'b0100; //R2
                            end
                            2'b10 : begin
                                reg_RF_RSel = 4'b0010; //R3
                            end
                            2'b11 : begin
                                reg_RF_RSel = 4'b0001; //R4
                            end
                        endcase
                    end
                    if (seq_counter == 2'b11) begin //
                        reg_RF_FunSel = 2'b01; // LOAD
                        reg_ARF_RegSel = 4'b0010; //SP
                        reg_ARF_FunSel = 2'b11; // INC
                    end
                end
                4'b1111 : begin //PUSH
                    if (seq_counter == 2'b10) begin //
                        reg_ARF_OutDSel <= 2'b01; // SP
                        case (RSEL)
                            2'b00 : begin
                                reg_RF_OutBSel = 3'b100; //R1
                            end
                            2'b01 : begin
                                reg_RF_OutBSel = 3'b101; //R2
                            end
                            2'b10 : begin
                                reg_RF_OutBSel = 3'b110; //R3
                            end
                            2'b11 : begin
                                reg_RF_OutBSel = 3'b111; //R4
                            end
                        endcase
                    reg_ALU_FunSel = 4'b0001; //Pass B
                    reg_Mem_WR = 1; // write
                    end
                    if (seq_counter == 2'b11) begin //
                        reg_ARF_RegSel = 4'b0010; //SP
                        reg_ARF_FunSel = 2'b10; // DEC
                    end
                end
            endcase
        end
    end

    reg update_ADDRESS_flag;
    reg update_SREGA_flag;
    reg update_SREGB_flag;
    reg update_DSTREG_flag;

    always @(posedge update_SREGA_flag) begin
        reg_MuxCSel = (SREGA > 4'd3) ? 1 : 0;
        #1;    
        case(SREGA)
            4'b0000 : begin 
                reg_RF_OutASel = 3'b100;
            end
            4'b0001 : begin 
                reg_RF_OutASel = 3'b101;
            end
            4'b0010 : begin 
                reg_RF_OutASel = 3'b110;
            end
            4'b0011 : begin 
                reg_RF_OutASel = 3'b111;
            end
            4'b0100 : begin 
                reg_ARF_OutCSel = 2'b01; //SP
            end
            4'b0101 : begin 
                reg_ARF_OutCSel = 2'b00; //AR
            end
            4'b0110 : begin 
                reg_ARF_OutCSel = 2'b11; // PC
            end
            4'b0111 : begin 
                reg_ARF_OutCSel = 2'b11; // PC
            end
        endcase
        #3;
        update_SREGA_flag <= 0;
    end
    always @(posedge update_SREGB_flag) begin
        
        case (SREGB)
        4'b0000 : begin 
            reg_RF_OutBSel = 3'b100;
        end
        4'b0001 : begin 
            reg_RF_OutBSel = 3'b101;
        end
        4'b0010 : begin 
            reg_RF_OutBSel = 3'b110;
        end
        4'b0011 : begin 
            reg_RF_OutBSel = 3'b111;
        end
        endcase
        #4;    

        update_SREGB_flag <= 0;
    end


    always @(posedge update_DSTREG_flag) begin

        if (DSTREG < 4'd4) begin
            reg_MuxASel = 0;
        end else begin
            reg_MuxBSel = 0;
        end

        reg_RF_FunSel = (DSTREG < 4'd4) ? 2'b01 : 2'bX;
        reg_ARF_FunSel = (DSTREG < 4'd4) ? 2'bX : 2'b01;
        #2.5;
        $display("DSTREG: %d", DSTREG);
        $display("reg_RF_FunSel: %d", reg_RF_FunSel);
        $display("reg_ARF_FunSel: %d", reg_ARF_FunSel);

        case(DSTREG)
            4'b0000 : begin 
                reg_RF_RSel = 4'b1000;
            end
            4'b0001 : begin 
                reg_RF_RSel = 4'b0100;
            end
            4'b0010 : begin 
                reg_RF_RSel = 4'b0010;
            end
            4'b0011 : begin 
                reg_RF_RSel = 4'b0001;
            end
            4'b0100 : begin 
                reg_ARF_RegSel = 4'b0010; // SP
            end
            4'b0101 : begin 
                reg_ARF_RegSel = 4'b0100;// AR
            end
            4'b0110 : begin 
                reg_ARF_RegSel = 4'b1000;// PC
            end
            4'b0111 : begin 
                reg_ARF_RegSel = 4'b1000; // PC
            end
        endcase
        update_DSTREG_flag <= 0;
    end
    reg update_SREG1_flag;
    always @(posedge update_SREG1_flag) begin
        case(SREG1)
            4'b0000 : begin 
                reg_RF_RSel = 4'b1000;
            end
            4'b0001 : begin 
                reg_RF_RSel = 4'b0100;
            end
            4'b0010 : begin 
                reg_RF_RSel = 4'b0010;
            end
            4'b0011 : begin 
                reg_RF_RSel = 4'b0001;
            end
            4'b0100 : begin 
                reg_ARF_RegSel = 4'b0010; // SP
            end
            4'b0101 : begin 
                reg_ARF_RegSel = 4'b0100;// AR
            end
            4'b0110 : begin 
                reg_ARF_RegSel = 4'b1000;// PC
            end
            4'b0111 : begin 
                reg_ARF_RegSel = 4'b1000; // PC
            end
        endcase
        update_SREG1_flag <= 0;
    end
endmodule

module IR(
    input E,
    input [1:0] FunSel,
    input [7:0] Input,
    input LH,
    output [15:0] IROut
    );
    reg [15:0] complete_IR = 16'd0;
    always @(*) begin
        if (E == 1) begin
            if (FunSel == 2'b00) begin
                complete_IR <= 16'd0;
            end
            if (FunSel == 2'b01) begin
                if (LH == 0) begin
                    complete_IR[7:0] <= Input;
                end
                if (LH == 1) begin
                    complete_IR[15:8] <= Input;
                end
            end
            if (FunSel == 2'b10) begin
                complete_IR <= complete_IR - 16'd1;
            end
            if (FunSel == 2'b11) begin
                complete_IR <= complete_IR + 16'd1;
            end
        end
        
    end
    assign IROut = complete_IR;
endmodule

module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
    );
    //Declaration oif the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o <= ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule

module MUX_2bit(
    select,
    input_1,
    input_2,
    out
    );
    input wire select;
    input wire [7:0]input_1;
    input wire [7:0]input_2;
    output reg [7:0]out;
    
    always @(*) begin
        case(select)
            1'b0: out <= input_1;
            1'b1: out <= input_2;
        endcase
    end
endmodule

module MUX_4bit(
    select,
    input_1,
    input_2,
    input_3,
    input_4,
    out
    );
    input wire [1:0]select;
    input wire [7:0]input_1;
    input wire [7:0]input_2;
    input wire [7:0]input_3;
    input wire [7:0]input_4;
    output reg [7:0]out;
    

    always @(*) begin
        case(select)
            2'b00: out <= input_1;
            2'b01: out <= input_2;
            2'b10: out <= input_3;
            2'b11: out <= input_4;
        endcase
    end
endmodule

module RF(
    input [7:0] Input, 
    input [2:0] O1Sel,
    input [2:0] O2Sel,
    input [1:0] FunSel,
    input [3:0] RSel,
    input [3:0] TSel,
    output reg [7:0] Output1,
    output reg [7:0] Output2
    );
    reg [7:0] R1, R2, R3, R4;
    reg [7:0] T1, T2, T3, T4;
    reg SET_R1, SET_R2, SET_R3, SET_R4, SET_T1, SET_T2, SET_T3, SET_T4;   
   
    always @(RSel,O1Sel,O2Sel,FunSel) begin
        case (RSel)
            4'b0000: begin 
            end
            4'b0001: begin
            SET_R4 <= 1'b1;
            end
            4'b0010: begin
            SET_R3 <= 1'b1;
            end
            4'b0011: begin
            SET_R3 <= 1'b1;
            SET_R4 <= 1'b1;
            end
            4'b0100: begin
            SET_R2 <= 1'b1;
            end
            4'b0101: begin
            SET_R2 <= 1'b1;
            SET_R4 <= 1'b1;
            end
            4'b0110: begin
            SET_R2 <= 1'b1;
            SET_R3 <= 1'b1;
            end
            4'b0111: begin
            SET_R2 <= 1'b1;
            SET_R3 <= 1'b1;
            SET_R4 <= 1'b1;
            end       
            4'b1000: begin 
            SET_R1 <= 1'b1;
            end
            4'b1001: begin
            SET_R1 <= 1'b1;
            SET_R4 <= 1'b1;
            end
            4'b1010: begin
            SET_R1 <= 1'b1;
            SET_R3 <= 1'b1;
            end
            4'b1011: begin
            SET_R1 <= 1'b1;
            SET_R3 <= 1'b1;
            SET_R4 <= 1'b1;
            end
            4'b1100: begin
            SET_R1 <= 1'b1;
            SET_R2 <= 1'b1;
            end
            4'b1101: begin
            SET_R1 <= 1'b1;
            SET_R2 <= 1'b1;
            SET_R4 <= 1'b1;
            end
            4'b1110: begin
            SET_R1 <= 1'b1;
            SET_R2 <= 1'b1;
            SET_R3 <= 1'b1;
            end
            4'b1111: begin
            SET_R1 <= 1'b1;
            SET_R2 <= 1'b1;
            SET_R3 <= 1'b1;
            SET_R4 <= 1'b1;
            end                                                                                                                                                        
        endcase
    end

      always @(RSel,O1Sel,O2Sel,FunSel) begin
        case (TSel)
            4'b0000: begin 
            end
            4'b0001: begin
            SET_T4 <= 1'b1;
            end
            4'b0010: begin
            SET_T3 <= 1'b1;
            end
            4'b0011: begin
            SET_T3 <= 1'b1;
            SET_T4 <= 1'b1;
            end
            4'b0100: begin
            SET_T2 <= 1'b1;
            end
            4'b0101: begin
            SET_T2 <= 1'b1;
            SET_T4 <= 1'b1;
            end
            4'b0110: begin
            SET_T2 <= 1'b1;
            SET_T3 <= 1'b1;
            end
            4'b0111: begin
            SET_T2 <= 1'b1;
            SET_T3 <= 1'b1;
            SET_T4 <= 1'b1;
            end       
            4'b1000: begin 
            SET_T1 <= 1'b1;
            end
            4'b1001: begin
            SET_T1 <= 1'b1;
            SET_T4 <= 1'b1;
            end
            4'b1010: begin
            SET_T1 <= 1'b1;
            SET_T3 <= 1'b1;
            end
            4'b1011: begin
            SET_T1 <= 1'b1;
            SET_T3 <= 1'b1;
            SET_T4 <= 1'b1;
            end
            4'b1100: begin
            SET_T1 <= 1'b1;
            SET_T2 <= 1'b1;
            end
            4'b1101: begin
            SET_T1 <= 1'b1;
            SET_T2 <= 1'b1;
            SET_T4 <= 1'b1;
            end
            4'b1110: begin
            SET_T1 <= 1'b1;
            SET_T2 <= 1'b1;
            SET_T3 <= 1'b1;
            end
            4'b1111: begin
            SET_T1 <= 1'b1;
            SET_T2 <= 1'b1;
            SET_T3 <= 1'b1;
            SET_T4 <= 1'b1;
            end                                                                                                                                                        
        endcase
    end

    always @(posedge SET_R1) begin
            if (FunSel == 2'b00) begin
                    R1 = 8'd0;
            end
            else if (FunSel == 2'b01) begin
                    R1 = Input;
            end
            else if (FunSel == 2'b10) begin
                    R1 = R1 - 8'd1;
            end     
            else if (FunSel == 2'b11) begin
                    R1 = R1 + 8'd1;
            end  
            SET_R1  <= 1'b0;
    end
    


    always @(posedge SET_R2) begin
            
        if (FunSel == 2'b00) begin
            R2 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            R2 = Input;
        end
        else if (FunSel == 2'b10) begin
            R2 = R2 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            R2 = R2 + 8'd1;
        end  
        SET_R2 <= 1'b0;
    end

    always @(posedge SET_R3) begin
            
        if (FunSel == 2'b00) begin
            R3 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            R3 = Input;
        end
        else if (FunSel == 2'b10) begin
            R3 = R3 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            R3 = R3 + 8'd1;
        end  
        SET_R3 <= 1'b0;
    end

    always @(posedge SET_R4) begin
            
        if (FunSel == 2'b00) begin
            R4 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            R4 = Input;
        end
        else if (FunSel == 2'b10) begin
            R4 = R4 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            R4 = R4 + 8'd1;
        end  
        SET_R4 <= 1'b0;
    end
    always @(negedge SET_R4) begin
        $display("R4 = %h", R4);
    end

    always @(posedge SET_T1) begin
            
        if (FunSel == 2'b00) begin
            T1 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            T1 = Input;
        end
        else if (FunSel == 2'b10) begin
            T1 = T1 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            T1 = T1 + 8'd1;
        end  
        SET_T1 <= 1'b0;
    end

    always @(posedge SET_T2) begin
            
        if (FunSel == 2'b00) begin
            T2 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            T2 = Input;
        end
        else if (FunSel == 2'b10) begin
            T2 = T2 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            T2 = T2 + 8'd1;
        end  
        SET_T2 <= 1'b0;
    end

    always @(posedge SET_T3) begin
            
        if (FunSel == 2'b00) begin
            T3 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            T3 = Input;
        end
        else if (FunSel == 2'b10) begin
            T3 = T3 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            T3 = T3 + 8'd1;
        end  
        SET_T3 <= 1'b0;
    end
        
    always @(posedge SET_T4) begin
        if (FunSel == 2'b00) begin
            T4 = 8'd0;
        end
        else if (FunSel == 2'b01) begin
            T4 = Input;
        end
        else if (FunSel == 2'b10) begin
            T4 = T4 - 8'd1;
        end     
        else if (FunSel == 2'b11) begin
            T4 = T4 + 8'd1;
        end  
        SET_T4 <= 1'b0;
    end

    
    always @ (*) begin
        case (O1Sel)
            3'b000: Output1 = T1;
            3'b001: Output1 = T2;
            3'b010: Output1 = T3;
            3'b011: Output1 = T4;
            3'b100: Output1 = R1;
            3'b101: Output1 = R2;
            3'b110: Output1 = R3;
            3'b111: Output1 = R4;
        endcase
        

        case (O2Sel)
        
            3'b000: Output2 = T1;
            3'b001: Output2 = T2;
            3'b010: Output2 = T3;
            3'b011: Output2 = T4;
            3'b100: Output2 = R1;
            3'b101: Output2 = R2;
            3'b110: Output2 = R3;
            3'b111: Output2 = R4;
        endcase
    end
    always @(R1, R2, R3, R4, O1Sel, O2Sel) begin
          $display("Change in RF: R1 = %h R2 = %h R3 = %h R4 = %h Output1 = %h Output2 = %h", R1, R2, R3, R4, Output1, Output2);
     end
endmodule
