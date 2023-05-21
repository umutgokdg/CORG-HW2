module IR (
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