module alu(       
      input  [31:0]   ALUin1,           
      input  [31:0]   ALUin2,           
      input  [3:0]    ALUctrl,     
      output [31:0]  ALUresult,                 
      output ALUzero  
   );  
      reg    [31:0] ALUresult;

 always @(*)  
 begin   
      case(ALUctrl)  
      4'b0000: ALUresult=(ALUin1 & ALUin2); 
      4'b0001: ALUresult=(ALUin1 | ALUin2);  
      4'b0010: ALUresult=ALUin1 + ALUin2;  
      4'b0110: ALUresult=ALUin1 - ALUin2;  
      4'b0111: if (ALUin1<ALUin2) begin
                  ALUresult=32'd1;  
                  end
                  else begin
                  ALUresult=32'd0;  
                  end  
     
      default: ALUresult=32'b0;   
      endcase  
 
      if ((ALUin1 - ALUin2)==0) begin
      ALUzero=1'b1;
         end
      else begin
      ALUzero=1'b0;   
      end
 end  
 
 endmodule  