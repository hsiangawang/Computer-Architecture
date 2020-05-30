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


	wire clk_i, rst_i;
    
//==== combinational circuit ==============================

assign clk_i = clk;
assign rst_i = proc_reset;
	
	always@ (*) begin  //cache access
		case(proc_addr[4:2])
		3'd0: begin
			dirty_bit = dirty_save[0];
			data_4word = block0_save;
			tag = tag0_save;
			valid_bit = valid_save[0];
		end
		3'd1: begin
			dirty_bit = dirty_save[1];
			data_4word = block1_save;
			tag = tag1_save;
			valid_bit = valid_save[1];
		end

		3'd2: begin
			dirty_bit = dirty_save[2];
			data_4word = block2_save;
			tag = tag2_save;
			valid_bit = valid_save[2];
		end

		3'd3: begin
			dirty_bit = dirty_save[3];
			data_4word = block3_save;
			tag = tag3_save;
			valid_bit = valid_save[3];
		end

		3'd4: begin
			dirty_bit = dirty_save[4];
			data_4word = block4_save;
			tag = tag4_save;
			valid_bit = valid_save[4];
		end

		3'd5: begin
			dirty_bit = dirty_save[5];
			data_4word = block5_save;
			tag = tag5_save;
			valid_bit = valid_save[5];
		end

		3'd6: begin
			dirty_bit = dirty_save[6];
			data_4word = block6_save;
			tag = tag6_save;
			valid_bit = valid_save[6];
		end


		3'd7: begin
			dirty_bit = dirty_save[7];
			data_4word = block7_save;
			tag = tag7_save;
			valid_bit = valid_save[7];
		end

		endcase
	end

	always @(valid_bit or tag or proc_addr or state) begin
		if(valid_bit && (tag == proc_addr[29:5]) && ~state[1] && ~state[0]) begin
			stall = 1'b0;
			hit = 1'b1;
		end
		else begin
			stall = 1'b1;
			hit = 1'b0;
		end
	end

	always @(*) begin
			case(proc_addr[1:0])
				2'd0: begin block = block_save; block[31:0] = proc_wdata; end
				2'd1: begin block = block_save; block[63:32] = proc_wdata; end
				2'd2: begin block = block_save; block[95:64] = proc_wdata; end
				2'd3: begin block = block_save; block[127:96] = proc_wdata; end
			endcase
	end
	/*
valid[0] = valid_save[0];
dirty[0] = dirty_save[0];
valid[1] = valid_save[1];
dirty[1] = dirty_save[1];
valid[2] = valid_save[2];
dirty[2] = dirty_save[2];
valid[3] = valid_save[3];
dirty[3] = dirty_save[3];
valid[4] = valid_save[4];
dirty[4] = dirty_save[4];
valid[5] = valid_save[5];
dirty[5] = dirty_save[5];
valid[6] = valid_save[6];
dirty[6] = dirty_save[6];
valid[7] = valid_save[7];
dirty[7] = dirty_save[7];

*/
	always @(*)begin	//to write data to cache
		case(state) 
		2'b00:begin
			tag0 = tag0_save;
			tag1 = tag1_save;
			tag2 = tag2_save;
			tag3 = tag3_save;
			tag4 = tag4_save;
			tag5 = tag5_save;
			tag6 = tag6_save;
			tag7 = tag7_save;
		if(proc_write && hit) begin
			case(proc_addr[4:2])
			3'd0: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[0] = 1'b1;
				dirty[0] = 1'b1;
				block0 = block;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
				end
			3'd1: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[1] = 1'b1; 
				dirty[1] = 1'b1;
				block1 = block;
				block0 = block0_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;				
			end
			3'd2: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[2] = 1'b1; 
				dirty[2] = 1'b1;
				block2 = block;
				block0 = block0_save;
				block1 = block1_save;
				block3 = block3_save;	
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd3: begin
				valid = valid_save;
				dirty = dirty_save;
				valid[3] = 1'b1; 
				dirty[3] = 1'b1;
				block3 = block;	
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;	
			end
			3'd4: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[4] = 1'b1; 
				dirty[4] = 1'b1;
				block4 = block; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd5: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[5] = 1'b1; 
				dirty[5] = 1'b1;
				block5 = block;
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block6 = block6_save;
				block7 = block7_save; 
			end
			3'd6: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[6] = 1'b1; 
				dirty[6] = 1'b1;
				block6 = block; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block7 = block7_save;
			end
			3'd7: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[7] = 1'b1; 
				dirty[7] = 1'b1;
				block7 = block; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
			end
			endcase
		end
		else begin
			valid = valid_save;
			dirty = dirty_save;
			block0 = block0_save;
			block1 = block1_save;
			block2 = block2_save;
			block3 = block3_save;
			block4 = block4_save;
			block5 = block5_save;
			block6 = block6_save;
			block7 = block7_save;
		end
		end
		2'b10:begin
			case(proc_addr[4:2])
			3'd0: begin 	
				valid = valid_save;
				dirty = dirty_save;
				valid[0] = 1'b1; 
				tag0 = proc_addr[29:5];
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				dirty[0] = 1'b0;
				block0 = mem_rdata;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;				 
			end
			3'd1: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[1] = 1'b1; 
				tag1 = proc_addr[29:5];
				tag0 = tag0_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				dirty[1] = 1'b0;
				block1 = mem_rdata; 
				block0 = block0_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd2: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[2] = 1'b1; 
				tag2 = proc_addr[29:5];
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				dirty[2] = 1'b0;
				block2 = mem_rdata; 
				block0 = block0_save;
				block1 = block1_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd3: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[3] = 1'b1; 
				tag3 = proc_addr[29:5];
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				dirty[3] = 1'b0;
				block3 = mem_rdata; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd4: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[4] = 1'b1; 
				tag4 = proc_addr[29:5];
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				dirty[4] = 1'b0;
				block4 = mem_rdata; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd5: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[5] = 1'b1; 
				tag5 = proc_addr[29:5];
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				dirty[5] = 1'b0;
				block5 = mem_rdata; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block6 = block6_save;
				block7 = block7_save;
			end
			3'd6: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[6] = 1'b1; 
				tag6 = proc_addr[29:5];
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag7 = tag7_save;
				dirty[6] = 1'b0;
				block6 = mem_rdata; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block7 = block7_save;
			end
			3'd7: begin 
				valid = valid_save;
				dirty = dirty_save;
				valid[7] = 1'b1; 
				tag7 = proc_addr[29:5];
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				dirty[7] = 1'b0;
				block7 = mem_rdata; 
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
			end
			endcase
		end
		default: begin
				valid = valid_save;
				dirty = dirty_save;
				tag0 = tag0_save;
				tag1 = tag1_save;
				tag2 = tag2_save;
				tag3 = tag3_save;
				tag4 = tag4_save;
				tag5 = tag5_save;
				tag6 = tag6_save;
				tag7 = tag7_save;
				block0 = block0_save;
				block1 = block1_save;
				block2 = block2_save;
				block3 = block3_save;
				block4 = block4_save;
				block5 = block5_save;
				block6 = block6_save;
				block7 = block7_save;	
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
					case(proc_addr[1:0])
						2'd0: proc_rdata = data_4word[31:0];
						2'd1: proc_rdata = data_4word[63:32];
						2'd2: proc_rdata = data_4word[95:64];
						2'd3: proc_rdata = data_4word[127:96];
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
			valid_save <= 8'b0;
			dirty_save <= 8'b0;  
			tag0_save <= 25'b0;  
			tag1_save <= 25'b0;  
			tag2_save <= 25'b0;  
			tag3_save <= 25'b0;  
			tag4_save <= 25'b0;  
			tag5_save <= 25'b0;  
			tag6_save <= 25'b0;  
			tag7_save <= 25'b0;  
		end
	    else begin
	    	state <= next_state;
			valid_save <= valid;
			dirty_save <= dirty;
			tag0_save <= tag0;
			tag1_save <= tag1;
			tag2_save <= tag2;
			tag3_save <= tag3;
			tag4_save <= tag4;
			tag5_save <= tag5;
			tag6_save <= tag6;
			tag7_save <= tag7;
	    end
	end

	always @(posedge clk_i or posedge rst_i) begin
		if(rst_i) begin
			block0_save <= 128'b0;
			block1_save <= 128'b0;
			block2_save <= 128'b0;
			block3_save <= 128'b0;
			block4_save <= 128'b0;
			block5_save <= 128'b0;
			block6_save <= 128'b0;
			block7_save <= 128'b0;	
			block_save <= 128'b0;
		end
		else begin
			block0_save <= block0;
			block1_save <= block1;
			block2_save <= block2;
			block3_save <= block3;
			block4_save <= block4;
			block5_save <= block5;
			block6_save <= block6;
			block7_save <= block7;
			case(proc_addr[4:2])
			3'd0: block_save <= block0;
			3'd1: block_save <= block1;
			3'd2: block_save <= block2;
			3'd3: block_save <= block3;
			3'd4: block_save <= block4;
			3'd5: block_save <= block5;
			3'd6: block_save <= block6;
			3'd7: block_save <= block7;
			endcase
		end
	end
endmodule
