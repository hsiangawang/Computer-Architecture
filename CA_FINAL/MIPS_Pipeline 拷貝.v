

module  MIPS_Pipeline (
    // control interface
    clk, 
    rst_n,
//----------I cache interface-------    
    ICACHE_ren,
    ICACHE_wen,
    ICACHE_addr,
    ICACHE_wdata,
    ICACHE_stall,
    ICACHE_rdata,
//----------D cache interface-------
    DCACHE_ren,
    DCACHE_wen,
    DCACHE_addr,
    DCACHE_wdata,
    DCACHE_stall,
    DCACHE_rdata
  );
//input and output
  input clk;
  input rst_n;

  output  ICACHE_ren,ICACHE_wen;
  output [31:0] ICACHE_wdata;
  input  [31:0] ICACHE_rdata;
  output [29:0] ICACHE_addr;
  input ICACHE_stall;
  

  output  DCACHE_ren,DCACHE_wen;
  output [31:0] DCACHE_wdata;
  input  [31:0] DCACHE_rdata;
  output [29:0] DCACHE_addr;
  input DCACHE_stall;
  

  // wire and reg

  reg [31:0] PCin,PCnext,PCplusfour,JumpAddr,BranchAddr_part,IR_addr_next,BranchAddr;
    wire  [5:0] instruction31_26;
    wire  [4:0] instruction25_21,instruction20_16,instruction15_11;
    wire  [15:0] instruction15_0;
    wire  [31:0] SignExtend; 
    wire  [5:0] instruction_field;
    wire  [3:0] ALUctrl;

    reg  [4:0] writereg;
    wire  [1:0] RegDST,MemtoReg,Jump;
    wire  [2:0] ALUOp;
    wire  RegWrite,Branch,MemRead,MemWrite,ALUSrc; //address
    reg  [31:0] write_back; //register file  IO
    wire [31:0]read_data1,read_data2;
    wire  [31:0] after_ALUSrc; //after mux to alu
    wire  [31:0] aluresult;
    wire branch_and_zero;
    wire zero;
    reg  [31:0] PC_store;
    reg   [31:0] MUX_Branch;


    wire IF_Flush,IFIDWrite;
    wire [31:0] PC_plus4;
    wire  [31:0] PC_plus4Reg;
    wire  [31:0] InstReg;

    reg IF_Flush_w;

    
    wire EXMemRead,PCWrite;
    wire HazMux_signal;
    wire read_data_equal;

    wire [2:0] WB;
    wire [2:0] M;
    wire [5:0] EX;
    wire  [2:0] EX_WB,MEM_WB;
    wire  [2:0] EX_M,MEM_M;
    wire  [5:0] EX_reg;
    wire  [4:0] EX_Rs,EX_Rt,EX_Rd;
    wire  [31:0] EX_data1,EX_data2;
    wire  [31:0] EX_pcplus4;
    wire  [31:0] EX_signextend;

    reg EX_MEM_RegWrite;
    reg MEM_WB_RegWrite;
    wire [4:0]MEM_Rd,WB_Rd;

    wire [1:0] Foward_A,Foward_B;

    reg [31:0] after_A_mux,after_B_mux;
    reg [31:0] MEM_WBdata;
    wire  [31:0] ALUOut_reg;

    reg [4:0] after_RegDst;
    wire [31:0] MEM_writedata,MEM_readdata;
    reg [31:0] IR_addr;
    wire  [31:0] Inst;

    wire [2:0] WB_WB;
    wire [31:0] WB_readdata,WB_ALUOut;
    wire PCSrc;

    wire [2:0] WB_after_detect;
    wire [2:0] MEM_after_detect;
    wire [5:0] EX_after_detect;
    
    reg stall_bothzero;

    wire [31:0] MEM_pcplus4,WB_pcplus4;

    wire [31:0] RegTest;

    wire [31:0] read_data2_hope;

    wire Foward_C,Foward_D;
    wire [31:0] after_C_MUX,after_D_MUX;

    wire [31:0] reg0,reg1,reg2,reg3,reg4,reg5,reg6,reg7,reg8,reg9,reg10,reg11,reg12,reg13,reg14,reg15,reg16,reg17,reg18
      ,reg19,reg20,reg21,reg22,reg23,reg24,reg25,reg26,reg27,reg28,reg29,reg30,reg31;

    reg     [31:0] reg_file [0:31];
    reg     [31:0] reg_file_next [0:31]; 
    integer i;

    //==========combinational part===============

    assign ICACHE_ren = 1'b1;
    assign ICACHE_wen = 1'b0;
    assign ICACHE_addr = IR_addr[31:2];
    assign ICACHE_wdata = 31'b0;
    assign Inst = ICACHE_rdata;

    always @(*) begin
      if ((ICACHE_stall)&&(DCACHE_stall)) begin
        
        stall_bothzero=0;
        
      end
      else if(ICACHE_stall)begin


        stall_bothzero=0;

      end
      else  begin
        
        stall_bothzero=1;

      end
    end


  assign PC_plus4 = IR_addr+32'd4;

  

  IFID IFID1(

      .IF_Flush(IF_Flush),
      .clk(clk),
      .IFIDWrite(IFIDWrite),
      .PC_plus4(PC_plus4),
      .PC_plus4Reg(PC_plus4Reg),
      .Inst(Inst),
      .InstReg(InstReg),
      .stall_bothzero(stall_bothzero),
      .rst_n(rst_n)
    );


    assign instruction31_26[5:0] = InstReg[31:26];
    assign instruction25_21[4:0] = InstReg[25:21]; //rs
    assign instruction20_16[4:0] = InstReg[20:16]; //rt
    assign instruction15_11[4:0] = InstReg[15:11]; //rd
    assign instruction15_0[15:0] = InstReg[15:0];



    Hazard_detection hazard_detection(
              .IDRegRS(instruction25_21),
              .IDRegRt(instruction20_16),
              .EXRegRt(EX_Rt),
              .EXMemRead(EX_M[2]),
              .PCWrite(PCWrite),
              .IFIDWrite(IFIDWrite),
              .HazMux(HazMux_signal)    //mux 
                      );



    control control_unit(
            .opcode(instruction31_26),  
                    .rst_n(rst_n),  
                    .Rs(instruction25_21),
                    .Rt(instruction20_16),
                    .Rd(instruction15_11),
                    .RegDST(RegDST),    //EX
                    .MemtoReg(MemtoReg), //WB
                    .Jump(Jump),
                    .Branch(Branch),    //M
                    .MemRead(MemRead),  //M
                    .MemWrite(MemWrite), //M
                    .ALUSrc(ALUSrc),     //EX
                    .RegWrite(RegWrite), //WB
                    .ALUOp(ALUOp)       //EX
      );

    assign WB = {MemtoReg,RegWrite} ;  //combine them
    assign M = {MemRead,MemWrite,Branch};
    assign EX = {RegDST,ALUSrc,ALUOp};

     assign WB_after_detect = (HazMux_signal)? WB:3'b0;
    assign MEM_after_detect= (HazMux_signal)? M:3'b0;
    assign EX_after_detect = (HazMux_signal)? EX:6'b0;

    always @(*) begin

      for (i=0;i<32;i=i+1)
    reg_file_next[i] = reg_file[i];

    if (WB_WB[0])
    reg_file_next [WB_Rd] = MEM_WBdata;
    
      end

     
      assign read_data1 = reg_file_next[instruction25_21];
      assign read_data2 = reg_file_next[instruction20_16];


  

  beq_foward beq_foward1(.RegWrite(EX_WB[0]),.instruction31_26(instruction31_26),.instruction25_21(instruction25_21),
                          .instruction20_16(instruction20_16),.EX_Rd(EX_Rd),.Foward_C(Foward_C),.Foward_D(Foward_D));

  assign after_C_MUX = (Foward_C)? aluresult:read_data1;
  assign after_D_MUX = (Foward_D)? aluresult:read_data2;

   assign read_data_equal = (after_C_MUX==after_D_MUX)?1'b1:1'b0;


  //assign read_data2_hope = (WB_WB[0]&&(WB_Rd==instruction20_16))? MEM_WBdata:read_data2;



  always @(*) begin //address
    
    PCplusfour = PC_plus4Reg;
    JumpAddr = {PCplusfour[31:28],InstReg[25:0],2'b0}; 

    BranchAddr_part = SignExtend << 2;
    BranchAddr = PCplusfour + BranchAddr_part;

    if(Branch&&read_data_equal) begin //PCSrc
    MUX_Branch = BranchAddr;
    end
    
    else begin
        
    MUX_Branch = PC_plus4;

    end

    case(Jump)
        2'b00:IR_addr_next=PC_plus4;   
        2'b01:IR_addr_next=MUX_Branch;
        2'b10:IR_addr_next=JumpAddr;
        2'b11:IR_addr_next=read_data1; //!

        endcase

    end

    


  assign PCSrc = (read_data_equal&&Branch)? 1'b1:1'b0;


  always @(*) begin
    
  if (PCSrc) begin
    IF_Flush_w=1;
  end

  else if ((PCSrc==0)&&(Jump==2'b10)) begin

    IF_Flush_w=1;
    
  end

  else if ((PCSrc==0)&&(Jump==2'b11)) begin

    IF_Flush_w=1;
    
  end

  else begin
    IF_Flush_w=0;
  end


  end


  assign IF_Flush=IF_Flush_w;
  //assign read_data_equal = (after_C_MUX==after_D_MUX)?1'b1:1'b0;

  assign SignExtend[31:0] = {instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15]
    ,instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],instruction15_0[15],
    instruction15_0[15:0]} ;
    

    IDEX IDEX1(

            .clk(clk),
            .WB(WB_after_detect),
            .M(MEM_after_detect),
            .EX(EX_after_detect),
            .RegRs(instruction25_21),
            .RegRt(instruction20_16),
            .RegRd(instruction15_11),
            .data1(read_data1),
            .data2(read_data2),
            .PC_plus4Reg(PC_plus4Reg),
            .sign_extend(SignExtend),
            .WB_reg(EX_WB),
            .M_reg(EX_M),
            .EX_reg(EX_reg),
            .RegRs_reg(EX_Rs),
            .RegRt_reg(EX_Rt),
            .RegRd_reg(EX_Rd),
            .data1_reg(EX_data1),
            .data2_reg(EX_data2),
            .EX_pcplus4(EX_pcplus4),
            .sign_extend_reg(EX_signextend),
            .stall_bothzero(stall_bothzero),
            .rst_n(rst_n)
        );

    assign instruction_field = EX_signextend[5:0];

     ALUcontrol alucontrol(.ALUctrl(ALUctrl),.ALUOp(EX_reg[2:0]),.func_field(instruction_field));

     fowarding_unit fowarding_unit1(
            .ID_EX_Rs(EX_Rs),
            .ID_EX_Rt(EX_Rt),
            .EX_MEM_RegWrite(MEM_WB[0]),
            .MEM_WB_RegWrite(WB_WB[0]),
            .EX_MEM_Rd(MEM_Rd),
            .MEM_WB_Rd(WB_Rd),
            .Foward_A(Foward_A),
            .Foward_B(Foward_B)
        );

     always @(*) begin
      case(Foward_A)

      2'b00:after_A_mux=EX_data1;
      2'b01:after_A_mux=MEM_WBdata;
      2'b10:after_A_mux=ALUOut_reg;
      default:after_A_mux=EX_data1;

      endcase
     end


     always @(*) begin
      case(Foward_B)

      2'b00:after_B_mux=EX_data2;
      2'b01:after_B_mux=MEM_WBdata;
      2'b10:after_B_mux=ALUOut_reg;
      default:after_B_mux=EX_data2;

      endcase
     end

      assign after_ALUSrc=(EX_reg[3]==0)? after_B_mux:EX_signextend;


     alu ALU(.ALUin1(after_A_mux),.ALUin2(after_ALUSrc),.ALUctrl(ALUctrl),.ALUresult(aluresult),.ALUzero(zero));

     always @(*) begin  //mux
    case(EX_reg[5:4])
    2'b00:after_RegDst[4:0]=EX_Rt;
    2'b01:after_RegDst[4:0]=EX_Rd;
    2'b10:after_RegDst[4:0]=5'b11111;
    default:after_RegDst[4:0]=5'b00000;
        endcase
    end



    EXMEM EXMEM1(

    .clk(clk),
    .WB(EX_WB),
    .M(EX_M),
    .ALUOut(aluresult),
    .RegRD(after_RegDst),
    .writedata(after_B_mux),
    .EX_pcplus4(EX_pcplus4),
    .WB_reg(MEM_WB),
    .M_reg(MEM_M),
    .ALUOut_reg(ALUOut_reg),
    .RegRD_reg(MEM_Rd),
    .Writedata_reg(MEM_writedata),
    .stall_bothzero(stall_bothzero),
    .MEM_pcplus4(MEM_pcplus4),
    .rst_n(rst_n)
    );

   

  assign DCACHE_ren = MEM_M[2];
  assign DCACHE_wen = MEM_M[1];
  assign DCACHE_addr = ALUOut_reg[31:2];
  assign DCACHE_wdata = MEM_writedata;
  assign MEM_readdata = DCACHE_rdata;

  MEMWB MEMWB1(.clk(clk),.WB(MEM_WB),.Memout(MEM_readdata),.ALUOut(ALUOut_reg),.RegRD(MEM_Rd),.MEM_pcplus4(MEM_pcplus4),
    .WBreg(WB_WB),.Memreg(WB_readdata),.ALUreg(WB_ALUOut),.RegRDreg(WB_Rd),.stall_bothzero(stall_bothzero),.WB_pcplus4(WB_pcplus4),.rst_n(rst_n));

  always @(*) begin

    case(WB_WB[2:1])
    2'b00:MEM_WBdata[31:0]=WB_ALUOut[31:0];
    2'b01:MEM_WBdata[31:0]=WB_readdata[31:0];
    2'b10:MEM_WBdata[31:0]=WB_pcplus4[31:0];   //if need to go with pipeline
    default:MEM_WBdata[31:0]=32'd0;
        endcase

    end




//========sequential part============

//==== sequential part ====================================

always @(posedge clk or negedge rst_n) begin
    if (~rst_n )begin     //pc write!!!!!!!!!!!!!!!!!!!!!!!!!!!!! dont forget
        IR_addr<=32'b0; 
         for(i = 0; i<32;i=i+1)
              reg_file[i] <= 32'b0;

    end
    else if(stall_bothzero&&PCWrite) begin
        IR_addr<=IR_addr_next;
        for(i = 0; i<32;i=i+1)
                reg_file[i] <= reg_file_next[i];
    end
end

//=========================================================
endmodule



 /* module register_file  
 (         clk,  
           rst_n,  
        
           RegWrite,  
             Reg_W, //address  
            RF_writedata,  //
      
             Reg_R1,  //address
             Reg_R2,  //address

           ReadData1,  //
           ReadData2,  //
           reg0,reg1,reg2,reg3,reg4,reg5,reg6,reg7,reg8,reg9,reg10,reg11,reg12,reg13,reg14,reg15,reg16,reg17,reg18
      ,reg19,reg20,reg21,reg22,reg23,reg24,reg25,reg26,reg27,reg28,reg29,reg30,reg31
 );  

      input     clk;  
      input     rst_n;  
        
      input     RegWrite;  
      input     [4:0]  Reg_W; //address  
      input     [31:0] RF_writedata;  //
      
      input     [4:0]  Reg_R1;  //address
      output    [31:0] ReadData1;  //
    
      input     [4:0]  Reg_R2;  //address
      output    [31:0] ReadData2 ; //

      output   [31:0] reg0,reg1,reg2,reg3,reg4,reg5,reg6,reg7,reg8,reg9,reg10,reg11,reg12,reg13,reg14,reg15,reg16,reg17,reg18
      ,reg19,reg20,reg21,reg22,reg23,reg24,reg25,reg26,reg27,reg28,reg29,reg30,reg31;


      reg     [31:0] reg_file [0:31];
      reg     [31:0] reg_file_next [0:31]; 
      integer i;

      reg [31:0] ReadData1_keep,ReadData2_keep,Regkeep;

     


      

      always @(*) begin

      if (~rst_n) begin
          for(i=0;i<31;i=i+1)
                reg_file_next[i]=32'b0;
          
        end

      else if (RegWrite)begin

          for(i=0;(i<31)&&(Reg_W!=i);i=i+1)begin

                reg_file_next[i]=reg_file[i];

              end


                reg_file_next[Reg_W]=RF_writedata;
        
      end

      else begin

         for(i=0;(i<31)&&(Reg_W!=i);i=i+1)begin

                reg_file_next[i]=reg_file[i];

              end
        reg_file_next[Reg_W]=reg_file[Reg_W];
      end
     
      end

     
      assign ReadData1 = reg_file_next[Reg_R1];
      assign ReadData2 = reg_file_next[Reg_R2];




      always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
          // reset
          for(i = 0; i<32;i=i+1)
              reg_file[i] <= 32'b0;
          
        end
        else  begin

              for(i = 0; i<32;i=i+1)
                reg_file[i] <= reg_file_next[i];
          
        end
      end
        
      


 endmodule   */

// CAwEDSD spring 2017
// 

module IFID(

    IF_Flush,
    clk,
    IFIDWrite,
    PC_plus4,
    PC_plus4Reg,
    Inst,
    InstReg,
    stall_bothzero,
    rst_n
);
    input IF_Flush;
    input [31:0] Inst;
    input [31:0] PC_plus4;
    input IFIDWrite,clk;
    output [31:0] InstReg;
    output [31:0] PC_plus4Reg;
    input stall_bothzero;
    input rst_n;

    reg [31:0] InstReg;
    reg [31:0] PC_plus4Reg;

    reg [31:0]Inst_keep,PC_plus4_keep;

    reg [31:0] IF_pcplus4,IF_Inst;

    wire both_ok;


     always @(*) begin

     Inst_keep = InstReg;
     PC_plus4_keep = PC_plus4Reg;

     if (IF_Flush) begin

    IF_pcplus4=31'b0;
    IF_Inst=31'b0;   

     end

     else begin
     
    IF_pcplus4 = PC_plus4;
    IF_Inst = Inst;

     end
      
     end

     assign both_ok = (stall_bothzero&&IFIDWrite)?1'b1:1'b0;
 

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
          InstReg<=32'b0;
          PC_plus4Reg<=32'b0;
        end

          else begin
            
            InstReg<=(both_ok)? IF_Inst:Inst_keep;
            PC_plus4Reg<=(both_ok)? IF_pcplus4:PC_plus4_keep;

          end
            
    end   

endmodule


  module Hazard_detection(

    IDRegRS,
    IDRegRt,
    EXRegRt,
    EXMemRead,
    PCWrite,
    IFIDWrite,
    HazMux 
);


    input[4:0] IDRegRS,IDRegRt,EXRegRt;
    input EXMemRead;
    output  reg PCWrite,IFIDWrite,HazMux;
    
   

    always@(*)begin 

    if (EXMemRead&((EXRegRt==IDRegRS)|(EXRegRt==IDRegRt))) begin

    PCWrite=0;
    IFIDWrite=0;
    HazMux=0; //let control signal be zero by mux
        
    end

    else begin

    PCWrite=1;
    IFIDWrite=1;
    HazMux=1; //let control signal be zero by mux
        
    end
    

    end
    
   

endmodule



module control(    input[5:0] opcode,  
                    input rst_n,  
                    input [4:0]Rs,Rt,Rd,
                    output [1:0] RegDST,MemtoReg,Jump,
                    output  Branch,MemRead,MemWrite,ALUSrc,RegWrite,
                    output [2:0]ALUOp                    
   );  
 reg [1:0] RegDST_tempt,MemtoReg_tempt,Jump_tempt;
 reg Branch_tempt,MemRead_tempt,MemWrite_tempt,ALUSrc_tempt,RegWrite_tempt;
 reg [2:0] ALUOp_tempt; // need three bit to determine the aluop 
 reg [4:0] Rs_r,Rt_r,Rd_r;

 always @(*)  
 begin  
Rs_r=Rs;
 Rt_r=Rt;
 Rd_r=Rd;
      if(rst_n==0) begin  
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt= 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;    
      end  

      else begin  
      case(opcode)   
      6'b000000:begin


      if((Rs_r==0)&&(Rt_r==0)&&(Rd_r==0))begin// R type  
                
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b100;  //not one is  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;  
                
                end

                else if((Rs_r!= 5'b0)&&(Rt_r==0)&&(Rd_r==0)) begin //jr !

                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b11;  //from read data_1   
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;  
                  
                end


                /*RegDST_tempt = 2'bxx;  
                MemtoReg_tempt = 2'bxx;  
                ALUOp_tempt = 3'bxxx;  
                Jump_tempt = 2'b11;  //from read data_1   
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'bx;  
                RegWrite_tempt = 1'b0;  */

                else begin

                // R type  
                RegDST_tempt = 2'b01;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b010;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b1;  
                
                end
                
              end

    6'b001000:begin //addi Itype

                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b011;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b1;    
        
                end

    6'b001100:begin //andi

                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b100;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b1;    
        
                end

    6'b001101:begin  // ori
                
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b101;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b1;    

                end

    6'b001010:begin //slti set on less than
                
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00; //not sure  
                ALUOp_tempt = 3'b110;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b1;    

                end

    6'b001110:begin //xori// give aluop 2'b11
                
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b111;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b1;    

                end
                

      6'b100011: begin// lw  
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b01;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b1;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b1;    
                end               
                 

      6'b101011: begin// sw  
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b1;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b0;  
                end

                /*

                RegDST_tempt = 2'bxx;  
                MemtoReg_tempt = 2'bxx;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b1;  
                ALUSrc_tempt = 1'b1;  
                RegWrite_tempt = 1'b0;  
                end


                */
                

      6'b000100: begin// beq  
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b001;  
                Jump_tempt = 2'b01;  
                Branch_tempt = 1'b1;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;  
                end

                /*

                RegDST_tempt = 2'bxx;  
                MemtoReg_tempt = 2'bxx;  
                ALUOp_tempt = 3'b001;  
                Jump_tempt = 2'b01;  
                Branch_tempt = 1'b1;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;  


                */

                     

      6'b000010: begin// j  
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b10;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;  
                end
                         
         
      
      6'b000011: begin// jal  
                RegDST_tempt = 2'b10;  
                MemtoReg_tempt = 2'b10;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b10;   //!
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b1;  
                end
                

     /*

      RegDST_tempt = 2'b10;  
                MemtoReg_tempt = 2'b10;  
                ALUOp_tempt = 3'bxxx;  
                Jump_tempt = 2'b10;   //!
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'bx;  
                RegWrite_tempt = 1'b1;  

     */

    6'b001001:begin //jalr
                
                RegDST_tempt = 2'b10;  
                MemtoReg_tempt = 2'b10;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b11;  //same as jump register
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b1;  

                end

                /*

                RegDST_tempt = 2'b10;  
                MemtoReg_tempt = 2'b10;  
                ALUOp_tempt = 3'bxxx;  
                Jump_tempt = 2'b11;  //same as jump register
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'bx;  
                RegWrite_tempt = 1'b1;  


                */
                   
     
      default:  begin
                RegDST_tempt = 2'b00;  
                MemtoReg_tempt = 2'b00;  
                ALUOp_tempt = 3'b000;  
                Jump_tempt = 2'b00;  
                Branch_tempt = 1'b0;  
                MemRead_tempt = 1'b0;  
                MemWrite_tempt = 1'b0;  
                ALUSrc_tempt = 1'b0;  
                RegWrite_tempt = 1'b0;   
                 end
      endcase  
      end  
 end  

assign RegDST = RegDST_tempt ;
assign MemtoReg = MemtoReg_tempt ;
assign ALUOp = ALUOp_tempt;
assign Jump = Jump_tempt ;
assign Branch = Branch_tempt;
assign MemRead = MemRead_tempt ;
assign MemWrite =MemWrite_tempt ;
assign ALUSrc = ALUSrc_tempt ;
assign RegWrite = RegWrite_tempt;


 endmodule 

module IDEX(

    clk,
    WB,
    M,
    EX,
    RegRs,
    RegRt,
    RegRd,
    data1,
    data2,
    sign_extend,
    PC_plus4Reg,
    WB_reg,
    M_reg,
    EX_reg,
    RegRs_reg,
    RegRt_reg,
    RegRd_reg,
    data1_reg,
    data2_reg,
    EX_pcplus4,
    sign_extend_reg,
    stall_bothzero,
    rst_n
);

  input clk;
  input [2:0] WB;
  input [2:0]  M;
  input [5:0] EX;
  input [4:0] RegRs;
  input [4:0] RegRt;
  input [4:0] RegRd;
  input [31:0] data1;
  input [31:0] data2;
  input [31:0] sign_extend;
  input [31:0] PC_plus4Reg;
  input stall_bothzero;
  input rst_n;


    output reg [2:0] WB_reg;
    output reg [2:0] M_reg;
    output reg [5:0] EX_reg;
    output reg [31:0] data1_reg,data2_reg,sign_extend_reg;
    output reg [4:0] RegRs_reg,RegRt_reg,RegRd_reg;
    output reg [31:0] EX_pcplus4;


    reg [2:0]WB_keep;
    reg [2:0]M_keep;
    reg [5:0]EX_keep;
    reg [31:0]data1_keep,data2_keep,sign_extend_keep;
    reg [4:0]RegRs_keep,RegRt_keep,RegRd_keep;
    reg [31:0] EX_pcplus4_keep;

    
     always @(*) begin

     WB_keep = WB_reg;
     M_keep = M_reg;
     EX_keep = EX_reg;
     data1_keep = data1_reg;
     data2_keep = data2_reg;
     sign_extend_keep = sign_extend_reg;
     RegRs_keep = RegRs_reg;
     RegRt_keep = RegRt_reg;
     RegRd_keep = RegRd_reg;
     EX_pcplus4_keep = EX_pcplus4;
      
     end


    always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin

        WB_reg <= 3'b0;
        M_reg <= 3'b0;
        EX_reg <= 6'b0;
        RegRs_reg <= 5'b0;
        RegRt_reg <= 5'b0;
        RegRd_reg <= 5'b0;
        data1_reg <= 31'b0;
        data2_reg <= 31'b0;
        sign_extend_reg <= 31'b0;
        EX_pcplus4 <= 31'b0;
      
    end

    else begin
      
        WB_reg <=(stall_bothzero)? WB:WB_keep;
        M_reg <= (stall_bothzero)?M:M_keep;
        EX_reg <= (stall_bothzero)?EX:EX_keep;
        RegRs_reg <= (stall_bothzero)?RegRs:RegRs_keep;
        RegRt_reg <= (stall_bothzero)?RegRt:RegRt_keep;
        RegRd_reg <= (stall_bothzero)?RegRd:RegRd_keep;
        data1_reg <= (stall_bothzero)?data1:data1_keep;
        data2_reg <= (stall_bothzero)?data2:data2_keep;
        sign_extend_reg <= (stall_bothzero)?sign_extend:sign_extend_keep;
        EX_pcplus4 <= (stall_bothzero)?PC_plus4Reg : EX_pcplus4_keep;
    end
       
    end

   

endmodule


  module ALUcontrol( ALUctrl, ALUOp, func_field);  
 
 output  [3:0] ALUctrl;  
 input  [2:0] ALUOp;   //3 bit
 input  [5:0] func_field;  
 
 wire [8:0] ALUcontrol_input;  // 9 bit
 reg  [3:0] alucontrol_tempt;

 assign ALUcontrol_input = {ALUOp,func_field};  

 always @(*) begin
 case (ALUOp)  

 3'b000:alucontrol_tempt=4'b0010;
 3'b001:alucontrol_tempt=4'b0110;
 3'b010:

        case(func_field)

        6'b100000:alucontrol_tempt=4'b0010;
        6'b100010:alucontrol_tempt=4'b0110;
        6'b100101:alucontrol_tempt=4'b0001;
        6'b100100:alucontrol_tempt=4'b0000;
        6'b101010:alucontrol_tempt=4'b0111;
        6'b000000:alucontrol_tempt=4'b0011;
        6'b000010:alucontrol_tempt=4'b0100;
        6'b000011:alucontrol_tempt=4'b1000;
        6'b100111:alucontrol_tempt=4'b0101;
        6'b100110:alucontrol_tempt=4'b1001;
        default:alucontrol_tempt=4'b0010;

        endcase
  3'b011:alucontrol_tempt=4'b0010;
  3'b100:alucontrol_tempt=4'b0000;
  3'b101:alucontrol_tempt=4'b0001;
  3'b110:alucontrol_tempt=4'b0111;


  /*9'b000xxxxxx:alucontrol_tempt=4'b0010; // lw sw
  9'b001xxxxxx:alucontrol_tempt=4'b0110; // beq
  9'b010100000:alucontrol_tempt=4'b0010; // add
  9'b010100010:alucontrol_tempt=4'b0110; // sub 
  9'b010100100:alucontrol_tempt=4'b0000; // and 
  9'b010100101:alucontrol_tempt=4'b0001; // or
  9'b010101010:alucontrol_tempt=4'b0111; // slt
  9'b011xxxxxx:alucontrol_tempt=4'b0010; // addi
  9'b100xxxxxx:alucontrol_tempt=4'b0000; // andi
  9'b101xxxxxx:alucontrol_tempt=4'b0001; // ori
  9'b110xxxxxx:alucontrol_tempt=4'b0111; // slti
  9'b010000000:alucontrol_tempt=4'b0011; // sll
  9'b010000010:alucontrol_tempt=4'b0100; // srl
  9'b010000011:alucontrol_tempt=4'b1000; // sra
  9'b010100111:alucontrol_tempt=4'b0101; // nor
  9'b010100110:alucontrol_tempt=4'b1001; // xor*/

  default:alucontrol_tempt=4'b0010;  //defult is add   
  endcase  
  
  end
  assign ALUctrl=alucontrol_tempt;
 endmodule  


 module alu(       
      input  [31:0]   ALUin1,           
      input  [31:0]   ALUin2,           
      input  [3:0]    ALUctrl,     
      output  [31:0]  ALUresult,                 
      output  ALUzero  
   );  
    reg [31:0]ALUresult_tempt;  
    reg ALUzero_tempt;

 always @(*)  
 begin   
      case(ALUctrl)  
      4'b0000: ALUresult_tempt=(ALUin1 & ALUin2); 
      4'b0001: ALUresult_tempt=(ALUin1 | ALUin2);  
      4'b0010: ALUresult_tempt=ALUin1 + ALUin2;
      4'b0011: ALUresult_tempt=ALUin1<<ALUin2; //sll
      4'b0100: ALUresult_tempt=ALUin1>>ALUin2; //srl !
      4'b0101: ALUresult_tempt= ~(ALUin1 | ALUin2); //nor
      4'b0110: ALUresult_tempt=ALUin1 - ALUin2;  
      4'b0111: if (ALUin1<ALUin2) begin
                  ALUresult_tempt=32'd1;  
                  end
                  else begin
                  ALUresult_tempt=32'd0;  
                  end  
      4'b1000: ALUresult_tempt=ALUin1>>>ALUin2; //sra
      4'b1001: ALUresult_tempt=(ALUin1^ALUin2); //xor
     
      default: ALUresult_tempt=32'b0;   
      endcase  
 
      if ((ALUin1 - ALUin2)==0) begin
      ALUzero_tempt=1'b1;
         end
      else begin
      ALUzero_tempt=1'b0;   
      end


 end  
  assign ALUresult=ALUresult_tempt;
  assign ALUzero=ALUzero_tempt;
 
 endmodule  



  module EXMEM(

    clk,
    WB,
    M,
    ALUOut,
    RegRD,
    writedata,
    EX_pcplus4,
    WB_reg,
    M_reg,
    ALUOut_reg,
    RegRD_reg,
    Writedata_reg,
    stall_bothzero,
    MEM_pcplus4,
    rst_n
    );
   

   input clk;
   input [2:0] WB;
   input [2:0] M;
   input [4:0] RegRD;
   input [31:0] ALUOut,writedata;
   input stall_bothzero;
   input [31:0]EX_pcplus4;
   input rst_n;

   output reg[2:0] WB_reg;
   output reg[2:0] M_reg;
   output reg[31:0] ALUOut_reg,Writedata_reg;
   output reg[4:0] RegRD_reg;
   output reg[31:0] MEM_pcplus4;

   reg [2:0] WB_keep;
   reg [2:0] M_keep;
   reg [4:0] RegRD_keep;
   reg [31:0] ALUOut_keep,writedata_keep;
   reg [31:0] MEM_pcplus4_keep;

   always @(*) begin
    WB_keep = WB_reg;
    M_keep = M_reg;
    RegRD_keep = RegRD_reg;
    ALUOut_keep = ALUOut_reg;
    writedata_keep = Writedata_reg;
    MEM_pcplus4_keep = MEM_pcplus4;
   end
    always@(posedge clk or negedge rst_n)
    begin
    if (~rst_n) begin
        WB_reg <= 3'b0;
        M_reg <= 3'b0;
        ALUOut_reg <= 32'b0;
        RegRD_reg <= 5'b0;
        Writedata_reg <= 32'b0;
        MEM_pcplus4 <= 32'b0;
        end
      else begin
        WB_reg <= (stall_bothzero)?WB:WB_keep;
        M_reg <= (stall_bothzero)?M:M_keep;
        ALUOut_reg <= (stall_bothzero)?ALUOut:ALUOut_keep;
        RegRD_reg <= (stall_bothzero)?RegRD:RegRD_keep;
        Writedata_reg <= (stall_bothzero)?writedata:writedata_keep;
        MEM_pcplus4 <= (stall_bothzero)?EX_pcplus4:MEM_pcplus4_keep;
          end    
    end
endmodule


  


  module MEMWB(clk,WB,Memout,ALUOut,RegRD,MEM_pcplus4,WBreg,Memreg,ALUreg,RegRDreg,stall_bothzero,WB_pcplus4,rst_n);



   input clk;
   input [2:0] WB;
   input [4:0] RegRD;
   input [31:0] Memout,ALUOut;
   input stall_bothzero;
   input [31:0] MEM_pcplus4;
   input rst_n;

   output [2:0] WBreg;
   output [31:0] Memreg,ALUreg;
   output [4:0] RegRDreg;
   output reg [31:0] WB_pcplus4;

   reg [2:0] WBreg;
   reg [31:0] Memreg,ALUreg;
   reg [4:0] RegRDreg;

   reg [2:0]WB_keep;
   reg [4:0]RegRD_keep; 
   reg [31:0]Memout_keep,ALUOut_keep;
   reg [31:0]WB_pcplus4_keep;

    

    always @(*) begin

    WB_keep = WBreg;
    RegRD_keep = RegRDreg;
    Memout_keep = Memreg;
    ALUOut_keep = ALUreg;
    WB_pcplus4_keep = WB_pcplus4;
      
    end

   
    always@(posedge clk or negedge rst_n)
    begin
        if (~rst_n) begin
        WBreg <= 3'b0;
        Memreg <= 32'b0;
        ALUreg <= 32'b0;
        RegRDreg <= 5'b0;
        WB_pcplus4 <= 32'b0;
        end
        else begin
        WBreg <= (stall_bothzero)?WB:WB_keep;
        Memreg <= (stall_bothzero)?Memout:Memout_keep;
        ALUreg <= (stall_bothzero)?ALUOut:ALUOut_keep;
        RegRDreg <= (stall_bothzero)?RegRD:RegRD_keep;
        WB_pcplus4 <= (stall_bothzero)?MEM_pcplus4:WB_pcplus4_keep;
        end
    end
endmodule

// CAwEDSD spring 2017
//sign extend

module fowarding_unit(
    ID_EX_Rs,
    ID_EX_Rt,
    EX_MEM_RegWrite,
    MEM_WB_RegWrite,
    EX_MEM_Rd,
    MEM_WB_Rd,
    Foward_A,
    Foward_B
);
    
    input [4:0]ID_EX_Rs;
    input [4:0]ID_EX_Rt;
    input EX_MEM_RegWrite,MEM_WB_RegWrite;
    input [4:0]EX_MEM_Rd;
    input [4:0]MEM_WB_Rd;
    output reg[1:0]Foward_A,Foward_B;

    reg EX_MEM_equal1,EX_MEM_equal2,MEM_WB_equal1,MEM_WB_equal2;

    always@(*)begin 

    EX_MEM_equal1=(EX_MEM_Rd==ID_EX_Rs)?1'b1:1'b0;
    EX_MEM_equal2=(EX_MEM_Rd==ID_EX_Rt)?1'b1:1'b0;
    MEM_WB_equal1=(MEM_WB_Rd==ID_EX_Rs)?1'b1:1'b0;
    MEM_WB_equal2=(MEM_WB_Rd==ID_EX_Rt)?1'b1:1'b0;

    if (EX_MEM_RegWrite&&(EX_MEM_Rd!=0)&&EX_MEM_equal1) begin

        Foward_A=2'b10;
        Foward_B=2'b00;
        
    end

    else if (EX_MEM_RegWrite&&(EX_MEM_Rd!=0)&&EX_MEM_equal2) begin

        Foward_A=2'b00;
        Foward_B=2'b10;
    end

    else if (MEM_WB_RegWrite&&(MEM_WB_Rd!=0)&&MEM_WB_equal1) begin

        Foward_A=2'b01;
        Foward_B=2'b00;
        
    end

    else if (MEM_WB_RegWrite&&(MEM_WB_Rd!=0)&&MEM_WB_equal2) begin

        Foward_A=2'b00;
        Foward_B=2'b01;
        
    end

    else if (EX_MEM_RegWrite&&MEM_WB_RegWrite&&(EX_MEM_Rd!=0)&&(MEM_WB_Rd!=0)&&EX_MEM_equal1&&MEM_WB_equal1) begin

        Foward_A=2'b10;
        Foward_B=2'b00;
        
    end

    else if (EX_MEM_RegWrite&&MEM_WB_RegWrite&&(EX_MEM_Rd!=0)&&(MEM_WB_Rd!=0)&&EX_MEM_equal2&&MEM_WB_equal2) begin

        Foward_A=2'b00;
        Foward_B=2'b10;
        
    end


    else begin

        Foward_A=2'b00;
        Foward_B=2'b00;

    end

    end
    
   

endmodule


module beq_foward (RegWrite,instruction31_26,instruction25_21,instruction20_16,EX_Rd,Foward_C,Foward_D);

    input RegWrite;
    input [5:0] instruction31_26;
    input [4:0] instruction25_21,instruction20_16,EX_Rd;
    output reg Foward_C,Foward_D;

    always @(*) begin
      if (RegWrite&&(instruction25_21==EX_Rd)&&(instruction31_26==6'b000100)) begin

        Foward_C=1;
        Foward_D=0;
        
      end

      else if(RegWrite&&(instruction20_16==EX_Rd)&&(instruction31_26==6'b000100)) begin

      Foward_C=0;
      Foward_D=1;

        
      end

      else begin

        Foward_C=0;
        Foward_D=0;

      end
    end

endmodule
  
  





