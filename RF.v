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
   

 always @(FunSel, R1, Input, posedge SET_R1) begin
        
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

   always @(FunSel, R2, Input, posedge SET_R2) begin
        
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

always @(FunSel, R3, Input, posedge SET_R3) begin
        
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

always @(FunSel, R4, Input, posedge SET_R4) begin
        
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

always @(FunSel, T1, Input, posedge SET_T1) begin
        
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

always @(FunSel, T2, Input, posedge SET_T2) begin
        
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

always @(FunSel, T3, Input, posedge SET_T3) begin
        
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
    
    always @(FunSel, T4, Input, posedge SET_T4) begin
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
endmodule