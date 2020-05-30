 module HSs18n_128x32  
 (  
      input     clk,
       
      input     [31:0]  addr_byte,   //address in byte
      input     [31:0] D, 
      input     CEN,
      input     OEN,
      input     WEN,
      output    [31:0] Q    //read out data       
 );  
      wire [29:0]addr_word;
      wire [6:0] A;
      assign addr_word[29:0]=addr_byte[31:2];
      assign  A[6:0]=addr_word[6:0] ; //<128

      integer i;  
      reg [31:0] data_mem [0:127];  
      
      initial begin  
           for(i=0;i<128;i=i+1)  
                data_mem[i] <= 32'b0;  
      end  

      always @(negedge clk) begin  
           if (CEN==0)  begin
                case(WEN)
                1'b0: data_mem[A]<=D;
                1'b1: if (OEN==0) begin
                  Q<=data_mem[A];
                end
                default:Q<=32'b0;
                endcase
      end  
      end

 endmodule   



