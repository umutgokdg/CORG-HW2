module MUX_4bit (
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