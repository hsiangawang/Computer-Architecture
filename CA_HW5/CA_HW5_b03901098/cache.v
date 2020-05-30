module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_wdata,
    proc_stall,
    proc_rdata,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output   reg      proc_stall;
    output  reg[31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output    reg     mem_read, mem_write;
    output  reg[27:0] mem_addr;
    output reg[127:0] mem_wdata;
    
//==== wire/reg definition ================================

    reg [24:0] tag;
    reg  [2:0] index;
    reg  [1:0] block_offset;
    reg  [29:0] proc_addr_r;
    reg  [5:0]  block_number;
    reg  [24:0] cache_tag,cache_tag_next; 
    reg [154:0] CACHE [7:0];
    reg [154:0] CACHE_next [7:0];

    reg [31:0]set_1,set_2,set_3,set_4; //for debug
    reg valid,dirty,dirty_next;
    reg [1:0]state,next_state;
    reg [7:0]dirty_all;
    reg equal;
    integer i;
    reg [154:0]block;

    
//==== combinational circuit ==============================



always @(*) begin

    tag=proc_addr[29:5]; //25 bit
    index=proc_addr[4:2];
    block_offset=proc_addr[1:0];
    valid=CACHE[index][154];
    dirty=CACHE[index][153];
    proc_stall=1;
    block=CACHE[index];
    mem_read=0;
    mem_write=0;
    mem_addr=0;
    mem_wdata=0;
    proc_rdata=0;
    

if (proc_read) begin//read

    if (block[154]) begin //valid

        if (tag==block[152:128]) begin //hit

         case(block_offset)
            2'b00:proc_rdata=block[31:0];
            2'b01:proc_rdata=block[63:32];
            2'b10:proc_rdata=block[95:64];
            2'b11:proc_rdata=block[127:96];
            endcase
            proc_stall=0;

                    set_1=CACHE[index][31:0];
                    set_2=CACHE[index][63:32];
                    set_3=CACHE[index][95:64];
                    set_4=CACHE[index][127:96];
        end//hit

        else begin //miss

         proc_stall = 1; 
                
                    if(~block[153]) begin
                        if(~mem_ready) begin
                            mem_read=1;
                            mem_write=0;
                            mem_addr=proc_addr[29:2];
                        end
                        
                        else begin
                            mem_read  = 0;
                            mem_write = 0;
                            block[127:96]   = mem_rdata[127:96];
                            block[95:64]    = mem_rdata[95:64];
                            block[63:32]    = mem_rdata[63:32];
                            block[31:0]     = mem_rdata[31:0];
                            block[152:128]  = proc_addr[29:5];
                          
                        end
                    end
                    else begin
                        
                        if(~mem_ready) begin
                            mem_write=1;
                            mem_read=0;
                            mem_addr={block[152:128],index}; 
                            mem_wdata=block;
                        end
                        
                        else begin
                            block[153] = 0;
                        end
                    end

        
            
        end//miss


        
    end//valid

    else begin//not valid

                 proc_stall = 1;
                if (~mem_ready) begin
                    mem_read  = 1;
                    mem_write = 0;
                    mem_addr  = proc_addr[29:2];
                end
                
                else begin
                    mem_read  = 0;
                    mem_write = 0;
                    block[127:96]   = mem_rdata[127:96];
                    block[95:64]    = mem_rdata[95:64];
                    block[63:32]    = mem_rdata[63:32];
                    block[31:0]     = mem_rdata[31:0];
                    block[152:128]  = proc_addr[29:5];
                    block[154]= 1;
                    block[153]= 0;
                end


                    set_1=CACHE[index][31:0];
                    set_2=CACHE[index][63:32];
                    set_3=CACHE[index][95:64];
                    set_4=CACHE[index][127:96];
    end//not valid


    
end//proc_read

else if (proc_write)begin//proc_write

    if (tag==block[152:128]) begin//write hit

    case(block_offset)
                    2'b00:block[31:0]=proc_wdata;
                    2'b01:block[63:32]=proc_wdata;
                    2'b10:block[95:64]=proc_wdata;
                    2'b11:block[127:96]=proc_wdata;

                    endcase

                    set_1=CACHE[index][31:0];
                    set_2=CACHE[index][63:32];
                    set_3=CACHE[index][95:64];
                    set_4=CACHE[index][127:96];

                    block[153]=1;
                    proc_stall=0;

        
    end//write bit

    else begin//write miss

        proc_stall = 1; 
                    if(~block[153]) begin
                        if(~mem_ready) begin
                            mem_read=1;
                            mem_write=0;
                            mem_addr=proc_addr[29:2];
                        end
                        
                        else begin
                            mem_read  = 0;
                            mem_write = 0;
                
                           block[127:96]    = mem_rdata[127:96];
                            block[95:64]    = mem_rdata[95:64];
                            block[63:32]    = mem_rdata[63:32];
                            block[31:0]     = mem_rdata[31:0];
                            block[152:128]  = proc_addr[29:5];
                            
                        end
                    end
                
                    else begin
                        
                        if(~mem_ready) begin
                            mem_read=0;
                            mem_write=1;
                            mem_addr={block[152:128],index}; 
                            mem_wdata=block;
                        end
                        
                        else begin
                            block[153] = 0;
                        end
                    end


        
        
    end//write miss


end//proc write

else begin
    proc_stall=1;
    mem_read=0;
    mem_addr=0;
    mem_wdata=0;
    proc_rdata=0;
    mem_write=0;
    mem_read=0;
end
    
end//always



//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset ) begin
    if( proc_reset ) begin
    
    for (i=0;i<8;i=i+1)
                CACHE[i] <= 155'b0; 
            
        end

    else begin

      CACHE[index] <= block;
    
    end
end

endmodule
