module IR (
    input E,
    input [1:0] FunSel,
    input [7:0] Input,
    input LH,
    output [15:0] IROut
    );
    reg [15:0] complete_IR;
    assign IROut = complete_IR;
    always @(LH, Input, FunSel, E) begin
        if (E == 1) begin
                if (FunSel == 2'b00) begin
                    complete_IR <= 16'd0;
                end
                if (FunSel == 2'b01) begin
                    if (LH == 0) begin
                        complete_IR[7:0] <= Input;
                    end
                    else begin
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
        /*
        $display("Input: %d", Input[7:0]);
        $display("complete high: %d", complete_IR[15:8]);
        $display("complete low: %d", complete_IR[7:0]);*/
    end
endmodule