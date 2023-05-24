`timescale 1ns / 1ps
`include "CPUSystem.v"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/05/2023 01:16:48 AM
// Design Name: 
// Module Name: TestBench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Project2Test();
    reg Clock, Reset;
    reg [7:0] T;
    
    always 
    begin
        repeat (360) begin
            Clock = 1; #5; Clock = 0; #5; // 10ns period
        end
        $finish;
    end
   
    CPUSystem _CPUSystem( 
            .Clock(Clock),
            .Reset(Reset),
            .T(T)
        );
endmodule
