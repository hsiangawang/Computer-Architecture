 module control(    input[5:0] opcode,  
                    input rst_n,  
                    output reg[1:0] RegDST,MemtoReg,ALUOp,Jump 
                    output reg Branch,MemRead,MemWrite,ALUSrc,RegWrite,                     
   );  
 always @(*)  
 begin  
      if(rst_n==0) begin  
                RegDST = 2'b00;  
                MemtoReg = 2'b00;  
                ALUOp = 2'b00;  
                Jump = 2'b00;  
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'b0;  
                RegWrite = 1'b0;    
      end  

      else begin  
      case(opcode)   
      6'b000000:  // R type  
                RegDST = 2'b01;  
                MemtoReg = 2'b00;  
                ALUOp = 2'b10;  
                Jump = 2'b00;  
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'b0;  
                RegWrite = 1'b1;  
                end  

      6'b100011: // lw  
                RegDST = 2'b00;  
                MemtoReg = 2'b01;  
                ALUOp = 2'b00;  
                Jump = 2'b00;  
                Branch = 1'b0;  
                MemRead = 1'b1;  
                MemWrite = 1'b0;  
                ALUSrc = 1'b1;  
                RegWrite = 1'b1;                   
                end  

      6'b101011: // sw  
                RegDST = 2'bxx;  
                MemtoReg = 2'bxx;  
                ALUOp = 2'b00;  
                Jump = 2'b00;  
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b1;  
                ALUSrc = 1'b1;  
                RegWrite = 1'b0;  
                end    

      6'b000100: // beq  
                RegDST = 2'bxx;  
                MemtoReg = 2'bxx;  
                ALUOp = 2'b01;  
                Jump = 2'b00;  
                Branch = 1'b1;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'b0;  
                RegWrite = 1'b0;  
                end       

      6'b000010: // j  
                RegDST = 2'b00;  
                MemtoReg = 2'b00;  
                ALUOp = 2'b00;  
                Jump = 2'b01;  
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'b0;  
                RegWrite = 1'b0;  
                end          
         
      
      6'b000011: // jal  
                RegDST = 2'b10;  
                MemtoReg = 2'b10;  
                ALUOp = 2'bxx;  
                Jump = 2'b01;   //!
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'bx;  
                RegWrite = 1'b1;  
                end 

      6'b000011: // jr
                RegDST = 2'bxx;  
                MemtoReg = 2'bxx;  
                ALUOp = 2'bxx;  
                Jump = 2'b10;  //from read data_1   
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'bx;  
                RegWrite = 1'b0;  
                end          
                   
     
      default: 
                RegDST = 2'b00;  
                MemtoReg = 2'b00;  
                ALUOp = 2'b00;  
                Jump = 2'b00;  
                Branch = 1'b0;  
                MemRead = 1'b0;  
                MemWrite = 1'b0;  
                ALUSrc = 1'b0;  
                RegWrite = 1'b0;   
                end  
      endcase  
      end  
 end  
 endmodule 