 module ALUcontrol( ALUctrl, ALUOp, func_field);  
 
 output [3:0] ALUctrl;  
 input  [1:0] ALUOp;  
 input  [5:0] func_field;  
 
 wire [7:0] ALUcontrol_input;  

 assign ALUcontrol_input = {ALUOp,func_field};  
 always @(*) begin
 casex (ALUcontrol_input)  
  8'b00xxxxxx:ALUctrl=4'b0010;
  8'b01xxxxxx:ALUctrl=4'b0110;
  8'b10100000:ALUctrl=4'b0010;
  8'b10100010:ALUctrl=4'b0110;
  8'b10100100:ALUctrl=4'b0000;
  8'b10100101:ALUctrl=4'b0001;
  8'b10101010:ALUctrl=4'b0111;
  default:ALUctrl=4'b0010;  //defult is add   
  endcase  
  end
 endmodule  