module ControlUnit(
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
    output MuxCSel,
    output clock
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


reg [3:0] seq_counter = 0;
assign seq = seq_counter;
always @(posedge clock) begin
    seq_counter <= seq_counter + 1;
end

always @(seq_counter) begin
        // Set default values for all control signals
        
    reg_RF_OutASel = 3'bX;
    reg_RF_OutBSel = 3'bX;
    reg_RF_FunSel = 2'bX;
    reg_RF_RSel = 4'bX;
    reg_RF_TSel = 4'bX;
    reg_ALU_FunSel = 4'bX;
    reg_ARF_OutCSel = 2'bX;
    reg_ARF_OutDSel = 2'bX;
    reg_ARF_FunSel = 2'bX;
    reg_ARF_RegSel = 4'bX;
    reg_IR_LH = 1'bX;
    reg_IR_Enable = 1'bX;
    reg_IR_Funsel = 2'bX;
    reg_Mem_WR = 1'bX;
    reg_Mem_CS = 1'bX;
    reg_MuxASel = 2'bX;
    reg_MuxBSel = 2'bX;
    reg_MuxCSel = 1'bX; 

    
    SREGA = (SREG1 > SREG2) ? SREG1 : SREG2;
    SREGB = (SREG1 > SREG2) ? SREG2 : SREG1;

    if (seq_counter == 4'b0000) begin // T0: AR <- PC // pc = 0
        reg_ARF_FunSel = 2'b11; // INC
        reg_ARF_RegSel = 4'b1000; // PC
        reg_ARF_OutDSel = 2'b11; // PC
        reg_IR_LH = 0;
        reg_IR_Enable = 1; // read before increment problem
        reg_IR_Funsel = 2'b01;
    end

    else if (seq_counter == 4'b0001) begin // T1: AR <- PC  // pc = 1
        reg_ARF_FunSel = 2'b11; // INC
        reg_ARF_RegSel = 4'b1000; // PC
        reg_ARF_OutDSel = 2'b11; // OUTB
        reg_IR_LH = 1;
        reg_IR_Enable = 1;
        reg_IR_Funsel = 2'b01;
    end

    else begin 
        case(OPCODE)
            4'b0000 : begin //AND
            if (seq_counter == 4'b0010) begin // T2: RF <- RF & RF
                update_SREGA_flag = 1;
                update_SREGB_flag = 1;
                reg_ALU_FunSel = 4'b0111;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0001 : begin //OR
            if (seq_counter == 4'b0010) begin // T2: RF <- RF | RF
                update_SREGA_flag = 1;
                update_SREGB_flag = 1;
                reg_ALU_FunSel = 4'b1000;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0010 : begin //NOT
            if (seq_counter == 4'b0010) begin // T2: RF <- RF'
                SREGA = SREG1;
                update_SREGA_flag = 1;
                reg_ALU_FunSel = 4'b0010;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0011 : begin //ADD
            if (seq_counter == 4'b0010) begin // T2: RF <- RF + RF
                update_SREGA_flag = 1;
                update_SREGB_flag = 1;
                reg_ALU_FunSel = 4'b0100;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0100 : begin //SUB
            if (seq_counter == 4'b0010) begin // T2: RF <- RF - RF
                update_SREGA_flag = 1;
                update_SREGB_flag = 1;
                reg_ALU_FunSel = 4'b0101;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0101 : begin //LSR
            if (seq_counter == 4'b0010) begin // T2: RF <- RF + 1
                SREGA = SREG1;
                update_SREGA_flag = 1;
                reg_ALU_FunSel = 4'b1100;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0110 : begin //LSL
            if (seq_counter == 4'b0010) begin // T2: RF <- RF - 1
                SREGA = SREG1;
                update_SREGA_flag = 1;
                reg_ALU_FunSel = 4'b1011;
                //t3
                update_DSTREG_flag = 1;
            end
            end
            4'b0111 : begin //INC
            if (seq_counter == 4'b0010) begin
                update_SREG1_flag = 1;
                if(SREG1 > 4'd3) begin
                    reg_ARF_FunSel = 2'b11; // INC
                end
                else begin
                    reg_RF_FunSel = 2'b11;
                end
            end
            if (seq_counter == 4'b0011) begin // T2: RF <- RF + 1
                SREGA = SREG1;
                update_SREGA_flag = 1;
                reg_ALU_FunSel = 4'b0000; 
                //t3
                update_DSTREG_flag = 0;
            end
            if (seq_counter == 4'b0100) begin
                update_SREG1_flag = 1;
                if(SREG1 > 4'd3) begin
                    reg_ARF_FunSel = 2'b10; // DEC
                end
                else begin
                    reg_RF_FunSel = 2'b10;
                end
            end
            end
            4'b1000 : begin //DEC
            if (seq_counter == 4'b0010) begin
                update_SREG1_flag = 1;
                if(SREG1 > 4'd3) begin
                    reg_ARF_FunSel = 2'b01; // DEC
                end
                else begin
                    reg_RF_FunSel = 2'b01;
                end
            end
            if (seq_counter == 4'b0011) begin // T2: RF <- RF + 1
                SREGA = SREG1;
                update_SREGA_flag = 1;
                reg_ALU_FunSel = 4'b0000;
                //t3
                update_DSTREG_flag = 0;
            end
            if (seq_counter == 4'b0100) begin
                update_SREG1_flag = 1;
                if(SREG1 > 4'd3) begin
                    reg_ARF_FunSel = 2'b11; // INC
                end
                else begin
                    reg_RF_FunSel = 2'b11; // INC
                end
            end
            end

        endcase
    end
end

reg update_SREGA_flag;
reg update_SREGB_flag;
reg update_DSTREG_flag;

always @(posedge update_SREGA_flag) begin
    reg_MuxCSel = (SREGA > 4'd3) ? 1 : 0;

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
    update_SREGB_flag <= 0;
end

always @(posedge update_DSTREG_flag) begin 
    reg_MuxASel = (DSTREG < 4'd4) ? 0 : 2'bX;
    reg_RF_FunSel = (DSTREG < 4'd4) ? 2'b01 : 2'bX;

    reg_MuxBSel = (DSTREG < 4'd4) ? 2'bX : 0;
    reg_ARF_FunSel = (DSTREG < 4'd4) ? 2'bX : 2'b01;
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
