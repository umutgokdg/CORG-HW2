module MUX_2bit (
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