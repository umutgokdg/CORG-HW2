module ALU (
    A,B,FunSel,OutALU,OutFlag,RESET,Clock
);
input wire [7:0] A;
input wire [7:0] B;
input wire RESET;
input wire Clock;
input wire [3:0] FunSel;
output wire [7:0] OutALU;
output wire [3:0] OutFlag;

reg [8:0] result;

reg ZERO = 1'bX;
reg SET_ZERO;
reg CARRY = 1'bX;
reg SET_CARRY;
reg NEGATIVE = 1'bX;
reg SET_NEGATIVE;
reg OVERFLOW = 1'bX;
reg SET_OVERFLOW;

//OVERFLOW SHOULD BE CALLED BEFORE CARRY

always @(A,B,FunSel,Clock) begin
    result  <= 9'b0;
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
            result <= A - B ; // A - 
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