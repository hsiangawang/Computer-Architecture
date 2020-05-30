	ROM128x32 i_rom(
		.addr(IR_addr[8:2]),
		.data(IR)
	);

	module ROM128x32 (
	addr,
	data
);
	input [6:0] addr;
	output [31:0] data;
	reg [31:0] data;
	reg [31:0] mem [0:127];
	
	integer i;
	initial begin
		// Initialize the instruction memory
		$readmemh ("instructions.txt", mem);
		//$display("Reading instruction memory......");
		//
		//for ( i=0; i<19; i=i+1 )
		//begin
		//	$display("mem[%d] = %h", i, mem[i]);
		//end
	end	
	
	always @(addr) data = mem[addr];
	
endmodule


	module instr_mem          
 (  
      input  [31:0]   IR_addr,  
      output [31:0]   IR  
 );  
      wire [31:0] IR;
      wire [6:0] mem_addr;
      wire [6:0]mem_addr=IR_addr[8:2];  
      assign IR=(IR_addr[31:0]<128)?mem[mem_addr[6:0]]: 16'd0;  
 
 endmodule

