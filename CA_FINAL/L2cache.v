module L2cache(
    clk,
    proc_reset,
    Icache_read,
    Icache_write,
    Icache_addr,
    Icache_wdata,
    Icache_ready,
    Icache_rdata,
    Dcache_read,
    Dcache_write,
    Dcache_addr,
    Dcache_wdata,
    Dcache_ready,
    Dcache_rdata,

    Imem_read,
    Imem_write,
    Imem_addr,
    Imem_rdata,
    Imem_wdata,
    Imem_ready,
    Dmem_read,
    Dmem_write,
    Dmem_addr,
    Dmem_rdata,
    Dmem_wdata,
    Dmem_ready
);	// output improvement
    
	parameter BLOCK_SIZE = 128;
	parameter TAG_SIZE = 28 - BLOCK_BIT;

	parameter BLOCK_NUM = 64;
	parameter BLOCK_BIT = 4;

	parameter WAY_NUM = 4;

	parameter IDLE = 3'd0;
	parameter I_WRITE = 3'd1;
	parameter I_FROMMEM = 3'd2;
	parameter I_READY = 3'd3;
	parameter D_WRITE = 3'd4;
	parameter D_FROMMEM = 3'd5;
	parameter D_READY = 3'd6;

//==== input/output definition ============================
    // processor interface
    input          clk;
    input		   proc_reset;
    input 		   Icache_read;
    input 		   Icache_write;
    input	[27:0] Icache_addr;
    input 	[127:0] Icache_wdata;
    output 		   Icache_ready;
    output  [127:0] Icache_rdata;

    input 		   Dcache_read;
    input 		   Dcache_write;
    input	[27:0] Dcache_addr;
    input 	[127:0] Dcache_wdata;
    output 		   Dcache_ready;
    output  [127:0] Dcache_rdata;

    // memory interface
    output			Imem_read;
    output 			Imem_write;
    input 			Imem_ready;
    input  [127:0]  Imem_rdata;
    output [27:0]	Imem_addr;
    output [127:0]  Imem_wdata;

    output 			Dmem_read;
    output 			Dmem_write;
    input 			Dmem_ready;
    input  [127:0]  Dmem_rdata;    
    output 	[27:0]	Dmem_addr;
    output [127:0]  Dmem_wdata;
    
   
    
//==== wire/reg definition ================================

	reg [BLOCK_SIZE-1:0] block;
	reg  [BLOCK_SIZE-1:0]block_save[BLOCK_NUM-1:0] ;
	reg [BLOCK_SIZE-1:0] data; 	//cache
	reg [BLOCK_NUM-1:0] valid_save, type_save;
	reg valid, type;
	reg [TAG_SIZE-1:0] tag;
	reg [TAG_SIZE-1:0] tag_save [BLOCK_NUM-1:0];

	reg [2:0] state, next_state;

	wire [BLOCK_BIT-1:0] I_index, D_index, set;
	reg [5:0] index, index1, index2, index3;

	reg [BLOCK_SIZE-1:0] ref_save[1:0]

	wire [5:0] idx1, idx2, idx3, idx4;
	wire [1:0] ref1, ref2, ref3;

	wire I_hit, D_hit;

	integer i;

    // type  0 => Instruction, type 1 => data
//==== combinational circuit ==============================


	//assign I_index = Imem_addr[5:0];
	//assign D_index = Dmem_addr[5:0];

	assign I_index = Imem_addr[3:0];
	assign D_index = Dmem_addr[3:0];

	assign I_hit = valid && (tag == Icache_addr[27:6]) && ~type;
	assign D_hit = valid && (tag == Dcache_addr[27:6]) && type;

	assign Icache_rdata = block;
	assign Dcache_rdata = block;

	assign Imem_wdata = block;
	assign Dmem_wdata = block;

	assign Imem_addr = Icache_addr;
	assign Dmem_addr = Dcache_addr;

	assign Icache_ready = ~state[2] && state[1] && state[0];
	assign Dcache_ready = state[2] && state[1] && ~state[0];


	assign Imem_read = ~state[2] && state[1] && state[0];
	assign Imem_write = ~state[2] && state[1] && ~state[0];
	assign Dmem_read = state[2] && ~state[1] && state[0];
	assign Dmem_write = state[2] && ~state[1] && ~state[0];

	assign set = (state[2])? D_index: I_index;

	assign idx1 = set << 2;
	assign idx2 = set << 2 + 1;
	assign idx3 = set << 2 + 2;
	assign idx4 = set << 2 + 3;

	assign ref1 = ref_save[idx1];
	assign ref2 = ref_save[idx2];
	assign ref3 = ref_save[idx3];

	always @(*) begin
		if(~valid_save[idx1] || (ref1 == 2'b00))begin
			index = idx1;
			index1 = idx2;
			index2 = idx3;
			index3 = idx4;
		end
		else if(~valid_save[idx2]|| (ref2 == 2'b00))begin
			index = idx2;
			index1 = idx1;
			index2 = idx3;
			index3 = idx4;
		end
		else if(~valid_save[idx3]|| (ref3 == 2'b00))begin
			index = idx3;
			index1 = idx1;
			index2 = idx2;
			index3 = idx4;
		end
		else begin
			index = idx4;
			index1 = idx1;
			index2 = idx2;
			index3 = idx3;
		end

	end

	always @(*) begin
		case(state)
			IDLE: begin
				case({Icache_read, Icache_write, I_hit, Dcache_read, Dcache_write, D_hit})
				6'b101000: next_state = I_READY;
				6'b100000: next_state = I_FROMMEM;
				6'b011000: next_state = I_WRITE;
				6'b010000: next_state = I_FROMMEM;
				6'b000101: next_state = D_READY;
				6'b000100: next_state = D_FROMMEM;
				6'b000011: next_state = D_WRITE;
				6'b000010: next_state = D_FROMMEM;
				default: next_state = IDLE;
				endcase
			end

			I_WRITE: begin 
				if(Imem_ready)
					next_state = I_READY;
				else
					next_state = I_WRITE;
			end

			I_FROMMEM:begin
				if(Imem_ready) begin
					if(Icache_read)
						next_state = I_READY;
					else 
						next_state = I_WRITE;
				end
				else
					next_state = I_FROMMEM;
			end
			I_READY: next_state = IDLE;

			D_WRITE: begin 
				if(Dmem_ready)
					next_state = D_READY;
				else
					next_state = D_WRITE;
			end

			D_FROMMEM:begin
				if(Dmem_ready) begin
					if(Dcache_read)
						next_state = D_READY;
					else 
						next_state = D_WRITE;
				end
				else
					next_state = D_FROMMEM;
			end
			D_READY: next_state = IDLE;

			default:
				next_state = IDLE;
		endcase
	end

	always @(*) begin	
		valid = valid_save[index];
		type = type_save[index];
		tag = tag_save[index];
		block = block_save[index];

		case(state)
		I_WRITE:begin
					valid = 1'b1;
					type = 1'b0;
					tag = Icache_addr[27:6];
					block = Icache_wdata;
		end
		I_FROMMEM:	begin
					valid = 1'b1;
					type = 1'b0;
					tag = Icache_addr[27:6];
					block = Imem_rdata;
		end
		D_WRITE:begin
					valid = 1'b1;
					type = 1'b1;
					tag = Dcache_addr[27:6];
					block = Dcache_wdata;
		end
		D_FROMMEM:begin
					valid = 1'b1;
					type = 1'b1;
					tag = Dcache_addr[27:6];
					block = Dmem_rdata;
		end
		endcase
	end





//==== sequential circuit =================================
	always@( posedge clk or posedge proc_reset ) begin
	    if( proc_reset ) begin
	    	state <= 3'b00;
			valid_save <= 256'b0;
			type_save <= 256'b0;


			for(i = 0; i <BLOCK_NUM; i = i+1) begin
			ref_save[i] <= 2'b0;
			tag_save[i] <= 25'b0;  
			block_save[i] <= 128'b0;
			end
		end
	    else begin
	    	state <= next_state;
	    	ref_save[index] <= 2'b11;
	    	ref_save[index1] <= ref1 - 1'b1;
	    	ref_save[index2] <= ref2 - 1'b1;
	    	ref_save[index3] <= ref3 - 1'b1;
	    	type_save[index] <= type;
			valid_save[index] <= valid;
			tag_save[index] <= tag;
			block_save[index] <= block;

	    end
	end


endmodule


/*
	always @(*) begin	//for state
		case(state) 
			2'b00:begin 	//Read or Write
				case({(valid && (proc_addr[29:5] == tag)),dirty})
				2'b00: next_state = 2'b10;
				2'b01: next_state = 2'b11;
				2'b10: next_state = 2'b00;
				2'b11: next_state = 2'b00;
				endcase
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

	always @(*) begin //for output
		case(proc_addr[1:0])
			2'd0: proc_rdata = block[31:0];
			2'd1: proc_rdata = block[63:32];
			2'd2: proc_rdata = block[95:64];
			2'd3: proc_rdata = block[127:96];
		endcase
	end

	assign proc_stall =  ~valid || ~(proc_addr[29:5] == tag ) || state[1];
	assign mem_wdata = block;
	assign mem_addr = state[0] ? {tag, index}:proc_addr[29:2];
	assign mem_read = state[1] && ~state[0];
	assign mem_write = state[1] && state [0];


	always @(*)begin	//to write data to cache
			valid = valid_save[index];
			dirty = dirty_save[index];
			block = block_save[index];
			tag = tag_save[index];
			
		case(state) 
		2'b00:begin
			if(proc_write && valid && (proc_addr[29:5] == tag )) begin
				dirty = 1'b1;
				valid = 1'b1;
				case(proc_addr[1:0])
				2'd0: block[31:0] = proc_wdata; 
				2'd1: block[63:32] = proc_wdata; 
				2'd2: block[95:64] = proc_wdata; 
				2'd3: block[127:96] = proc_wdata; 
				endcase
			end
		end
		

		2'b10:begin
			valid = 1'b1;
			tag = proc_addr[29:5];
			dirty = 1'b0;
			block = mem_rdata;
		end

	
		endcase
	
	end

*/	

	

	

	


