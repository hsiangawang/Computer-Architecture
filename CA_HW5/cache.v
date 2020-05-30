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
    output reg        proc_stall;
    output reg [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output reg     mem_read, mem_write;
    output reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================

	reg [127:0] block0, block1, block2, block3, block4, block5, block6, block7, block; 	//cache
	reg [7:0] valid, valid_save;
	reg [7:0] dirty, dirty_save;
	reg [24:0] tag0, tag1, tag2, tag3, tag4, tag5, tag6, tag7;
	reg [24:0] tag0_save, tag1_save, tag2_save, tag3_save, tag4_save, tag5_save, tag6_save, tag7_save;

	reg hit, stall, dirty_bit, valid_bit;
	reg [24:0] tag, tag_save;
	reg [127:0] data_4word;

	reg [1:0] state, next_state;

	reg [127:0] mem_data;
	reg [127:0] block0_save, block1_save,block2_save,block3_save,block4_save,block5_save,block6_save,block7_save;
	reg [127:0] block_save;

	reg [154:0]CACHE[7:0];
	reg [154:0]CACHE_next[7:0];
	reg [2:0] index;
	reg [1:0] block_offset;
	reg valid_1,dirty_1;
	reg cache_tag;
	reg [127:0]data,data_next;

	integer i;

    
//==== combinational circuit ==============================

always @(*) begin

    tag=proc_addr[29:5]; //25 bit
    index=proc_addr[4:2];
    block_offset=proc_addr[1:0];
    valid_1=CACHE[index][154];
    dirty_1=CACHE[index][153];
    cache_tag=CACHE[index][152:128];

    end

	
	/*always@ (*) begin  //cache access
		case(index)
		3'b000: begin
			dirty_bit = dirty_save[0];
			data_4word = block0_save;
			tag = tag0_save;
			valid_bit = valid_save[0];
		end
		3'b001: begin
			dirty_bit = dirty_save[1];
			data_4word = block1_save;
			tag = tag1_save;
			valid_bit = valid_save[1];
		end

		3'b010: begin
			dirty_bit = dirty_save[2];
			data_4word = block2_save;
			tag = tag2_save;
			valid_bit = valid_save[2];
		end

		3'b011: begin
			dirty_bit = dirty_save[3];
			data_4word = block3_save;
			tag = tag3_save;
			valid_bit = valid_save[3];
		end

		3'b100: begin
			dirty_bit = dirty_save[4];
			data_4word = block4_save;
			tag = tag4_save;
			valid_bit = valid_save[4];
		end

		3'b101: begin
			dirty_bit = dirty_save[5];
			data_4word = block5_save;
			tag = tag5_save;
			valid_bit = valid_save[5];
		end

		3'b110: begin
			dirty_bit = dirty_save[6];
			data_4word = block6_save;
			tag = tag6_save;
			valid_bit = valid_save[6];
		end


		3'b111: begin
			dirty_bit = dirty_save[7];
			data_4word = block7_save;
			tag = tag7_save;
			valid_bit = valid_save[7];
		end

		endcase
	end*/

	always @(valid_1 or cache_tag or proc_addr or state) begin
		if(valid_1 && (tag == proc_addr[29:5]) && ~state[1] && ~state[0]) begin
			stall = 1'b0;
			hit = 1'b1;
		end
		else begin
			stall = 1'b1;
			hit = 1'b0;
		end
	end

	always @(*) begin
			case(block_offset)
				2'b00: data_next[31:0]=proc_wdata;
				2'b01: data_next[63:32]=proc_wdata;
				2'b10: data_next[95:64]=proc_wdata;
				2'b11: data_next[127:96]=proc_wdata;
			endcase
	end

	always @(*)begin	//to write data to cache
		case(state) 
		2'b00:begin
			
		if(proc_write && hit) begin
			case(index)
			3'b000: begin 
				CACHE_next[0][154]=1;
				CACHE_next[0][153]=1;
				CACHE_next[0][127:0]=data_next;
				end
			3'b001: begin 
				CACHE_next[1][154]=1;
				CACHE_next[1][153]=1;
				CACHE_next[1][127:0]=data_next;
						
			end
			3'b010: begin 
				CACHE_next[2][154]=1;
				CACHE_next[2][153]=1;
				CACHE_next[2][127:0]=data_next;
			end
			3'b011: begin
				CACHE_next[3][154]=1;
				CACHE_next[3][153]=1;
				CACHE_next[3][127:0]=data_next;
			end
			3'b100: begin 
				CACHE_next[4][154]=1;
				CACHE_next[4][153]=1;
				CACHE_next[4][127:0]=data_next;
				
			end
			3'b101: begin 
				CACHE_next[5][154]=1;
				CACHE_next[5][153]=1;
				CACHE_next[5][127:0]=data_next;
			end
			3'b110: begin 
				CACHE_next[6][154]=1;
				CACHE_next[6][153]=1;
				CACHE_next[6][127:0]=data_next;
			end
			3'b111: begin 
				CACHE_next[7][154]=1;
				CACHE_next[7][153]=1;
				CACHE_next[7][127:0]=data_next;
			end
			endcase
		end
		else begin
			CACHE_next[index][154:0]=CACHE[index][154:0];
		end
		end
		2'b10:begin
			case(index)
			3'b000: begin 	
				CACHE_next[0][154]=1;
				CACHE_next[0][153]=0;
				CACHE_next[0][152:128]=proc_addr[29:5];
				CACHE_next[0][127:0]=mem_rdata;			 
			end
			3'b001: begin 
				CACHE_next[1][154]=1;
				CACHE_next[1][153]=0;
				CACHE_next[1][152:128]=proc_addr[29:5];
				CACHE_next[1][127:0]=mem_rdata;		
				
			end
			3'b010: begin 
				CACHE_next[2][154]=1;
				CACHE_next[2][153]=0;
				CACHE_next[2][152:128]=proc_addr[29:5];
				CACHE_next[2][127:0]=mem_rdata;		
			end
			3'b011: begin 
				CACHE_next[3][154]=1;
				CACHE_next[3][153]=0;
				CACHE_next[3][152:128]=proc_addr[29:5];
				CACHE_next[3][127:0]=mem_rdata;		
			end
			3'b100: begin 
				CACHE_next[4][154]=1;
				CACHE_next[4][153]=0;
				CACHE_next[4][152:128]=proc_addr[29:5];
				CACHE_next[4][127:0]=mem_rdata;		
			end
			3'b101: begin 
				CACHE_next[5][154]=1;
				CACHE_next[5][153]=0;
				CACHE_next[5][152:128]=proc_addr[29:5];
				CACHE_next[5][127:0]=mem_rdata;		
			end
			3'b110: begin 
				CACHE_next[6][154]=1;
				CACHE_next[6][153]=0;
				CACHE_next[6][152:128]=proc_addr[29:5];
				CACHE_next[6][127:0]=mem_rdata;		
			end
			3'b111: begin 
				CACHE_next[7][154]=1;
				CACHE_next[7][153]=0;
				CACHE_next[7][152:128]=proc_addr[29:5];
				CACHE_next[7][127:0]=mem_rdata;		
			end
			endcase
		end
		default: begin
				CACHE_next[index][154:0]=CACHE[index][154:0];
				
		end
		endcase
	
	end

	always @(*) begin	//for state
		case(state) 
			2'b00:begin 	//Read or Write
				if(hit) begin
					next_state = 2'b00;
				end
				else if(dirty) begin
					next_state = 2'b11;
				end
				else begin
					next_state = 2'b10;
				end
			end

			2'b01:begin
				next_state = 2'b00;
			end

			2'b10:begin	//Write 4-word data from memory to cache
				if(mem_ready) begin
					next_state = 2'b00;
				end
				else begin
					next_state = 2'b10;
				end
			end
			2'b11:begin	//Write back
				if(mem_ready) begin
					next_state = 2'b10;
				end
				else begin
					next_state = 2'b11;
				end
			end
		endcase
	end

	always @(*) begin	//for output
		case(state)	
			2'b00:begin	//Read or Write
				proc_stall = stall;
				mem_read = 1'b0;
				mem_write = 1'b0;
				mem_addr = 28'b0;
				mem_wdata = 128'b0;
				if(proc_read) begin
					case(block_offset)
						2'b00: proc_rdata = CACHE[index][31:0];
						2'b01: proc_rdata = CACHE[index][63:32];
						2'b10: proc_rdata = CACHE[index][95:64];
						2'b11: proc_rdata = CACHE[index][127:96];
					endcase
				end
				else begin
					proc_rdata = 32'b0;
				end
			end

			2'b01: begin
				proc_stall = 1'b0;
				proc_rdata = 32'b0;
				mem_read = 1'b0;
				mem_write = 1'b0;
				mem_addr = 28'b0;
				mem_wdata = 128'b0;
			end
			2'b10:begin	//Write data from memory to cahce
				proc_stall = 1'b1;
				proc_rdata = 32'b0;
				mem_read = 1'b1;
				mem_write = 1'b0;
				mem_addr = proc_addr[29:2];
				mem_wdata = 128'b0;
			end

			2'b11:begin	//Write back
				proc_stall = 1'b1;
				proc_rdata = 32'b0;
				mem_read = 1'b0;
				mem_write = 1'b1;
				mem_addr = {tag, proc_addr[4:2]};
				mem_wdata = data_4word;
			end
		endcase
	end


	

	



//==== sequential circuit =================================
	always@( posedge clk or posedge proc_reset ) begin
	    if( proc_reset ) begin
	    	state <= 2'b00;
	    	data<=128'b0;
	    	for(i=0;i<8;i=i+1)begin
            CACHE[i] <= 155'b0;
    		end
			
		end
	    else begin
	    	state <= next_state;
	    	data<=data_next;
	    	for(i=0;i<8;i=i+1)begin
            CACHE[i] <= CACHE_next[i];
   			 end
			
	    end
	end

	
endmodule
