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
               $display("PC Input = %d", Input);
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

endmodule