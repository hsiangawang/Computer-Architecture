
module register_file  
 (    input     clk,  
      input     rst_n,  
        
      input     Regwrite,  
      input     [5:0]  Reg_W, //address  
      input     [31:0] RF_writedata,  //寫回來的資料
      
      input     [5:0]  Reg_R1,  //address
      output    [31:0] ReadData1,  //讀出去的資料
    
      input     [5:0]  Reg_2,  //address
      output    [31:0] ReadData2  //讀出去的資料
 );  
      reg     [31:0] reg_file [0:127];  
      integer i;
        
      always @ (posedge clk or negedge rst_n) begin  
           if(!rst_n) begin  
                for(i=0;i<128;i=i+1)
                reg_file[i]=32'b0;
           end  
           else begin  
                if(Regwrite) begin  
                     reg_file[Reg_W] <= RF_writedata;  
                end  
           end  
      end  
      assign ReadData1 = (Reg_R1 == 0)? 15'b0 : reg_file[Reg_R1];  
      assign ReadData2 = (Reg_R2 == 0)? 15'b0 : reg_file[Reg_R2];  
 endmodule   