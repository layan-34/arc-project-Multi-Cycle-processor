`timescale 1ns / 1ps


module datapath (
  
    input wire clk,
    input wire rst,
    input wire [31:0] instruction_in,
    input wire [31:0] mem_data_in,
    input wire [5:0] opcode,
    input wire [3:0] rd, rs, rt,
    input wire [13:0] immediate,
    input wire [1:0] pcSrc, regDest, readRd, aluSrc, wbData,
    input wire extOp, regWr, disablePC,
    input wire [3:0] aluOp,
    output wire [31:0] pc_address,
    output wire [31:0] mem_address,
    output wire [31:0] mem_data_out,
    output wire alu_zero, alu_negative, alu_positive,
	 output wire [31:0] instruction_out, 
    output wire [31:0] alu_result_out,   
    output wire [31:0] read_data1_out,   // Register file output 1
    output wire [31:0] read_data2_out,   // Register file output 2
    output wire [31:0] write_data_out,   // Write-back data
    output wire [3:0] write_reg_out      // Write register address
);

    // Program Counter signals
    wire [31:0] pc_current, pc_next;
    
    // Register File signals
    wire [3:0] read_reg1, read_reg2, write_reg;
    wire [31:0] read_data1, read_data2, write_data;
    wire [31:0] busA, busB;
    
    // ALU signals
    wire [31:0] alu_input1, alu_input2, alu_result;
    
    // Sign/Zero Extender signals
    wire [31:0] extended_immediate;
    
    // Branch/Jump signals
    wire [31:0] branch_target, jump_target;
    wire [31:0] pc_plus_one;
    
    // Pipeline register signals
    wire [31:0] if_id_pc, if_id_instruction;
    wire [31:0] id_ex_pc, id_ex_busA, id_ex_busB, id_ex_immediate;
    wire [31:0] ex_mem_alu_result, ex_mem_busB;
    wire [31:0] mem_wb_alu_result, mem_wb_mem_data;
    
    // PC increment
    assign pc_plus_one = pc_current + 4;
    
    // Branch target calculation
    assign branch_target = pc_current + extended_immediate;
    
    // Jump register target
    assign jump_target = read_data1;

    // Program Counter (using flip-flop)
    flipflop32 pc_register (
        .clk(clk),
        .writeEn(~disablePC),
        .in(pc_next),
        .out(pc_current),
        .reset(rst)
    );
    
    // PC Source Multiplexer (4-to-1)
    mux432b pc_mux (
        .d0(pc_plus_one),
        .d1(branch_target),
        .d2(jump_target),
        .d3(branch_target),
        .s(pcSrc),
        .y(pc_next)
    );

    // IF/ID Pipeline Register
    flipflop32 if_id_pc_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(pc_current),
        .out(if_id_pc),
        .reset(rst)
    );
    
    flipflop32 if_id_instruction_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(instruction_in),
        .out(if_id_instruction),
        .reset(rst)
    );

    // Register destination multiplexer
    mux4 reg_dest_mux (
        .d0(rd),
        .d1(rd + 4'd1),
        .d2(4'd14),
        .d3(4'd15),
        .s(regDest), 
        .y(write_reg)
    );
    
    // Read register multiplexers
    mux4 read_reg2_mux (
        .d0(rt),
        .d1(rd),
        .d2(rd+4'd1),
        .d3(rt),
        .s(readRd),
        .y(read_reg2)
    );
    
    assign read_reg1 = rs;

    // Register File using flip-flops
    wire [31:0] reg_write_data [15:0];
    wire [15:0] reg_write_enable;
    
    // Generate write enable signals for each register
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : reg_write_enable_gen
            assign reg_write_enable[i] = regWr && (write_reg == i) && (write_reg != 4'd15);
        end
    endgenerate
    
    // Register file using flip-flops
    generate
        for (i = 0; i < 16; i = i + 1) begin : register_file
            flipflop32 reg_inst (
                .clk(clk),
                .writeEn(reg_write_enable[i]),
                .in(write_data),
                .out(reg_write_data[i]),
                .reset(rst)
            );
        end
    endgenerate
    
    // Register file read operations
    assign read_data1 = (read_reg1 == 4'd15) ? pc_current : reg_write_data[read_reg1];
    assign read_data2 = (read_reg2 == 4'd15) ? pc_current : reg_write_data[read_reg2];

    // ID/EX Pipeline Registers
    flipflop32 id_ex_pc_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(if_id_pc),
        .out(id_ex_pc),
        .reset(rst)
    );
    
    flipflop32 id_ex_busA_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(read_data1),
        .out(id_ex_busA),
        .reset(rst)
    );
    
    flipflop32 id_ex_busB_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(read_data2),
        .out(id_ex_busB),
        .reset(rst)
    );
    
    flipflop32 id_ex_immediate_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(extended_immediate),
        .out(id_ex_immediate),
        .reset(rst)
    );

    // Sign/Zero Extender
    extender ext_unit (
        .imm_in(immediate),
        .ext_ctrl(extOp),
        .imm_out(extended_immediate)
    );

    // ALU Input 1
    assign alu_input1 = id_ex_busA;
    
    // ALU Input 2 Multiplexer
    mux432b alu_src_mux (
        .d0(id_ex_busB),
        .d1(id_ex_immediate),
        .d2(id_ex_immediate+ 32'd1),
        .d3(32'd0),
        .s(aluSrc),
        .y(alu_input2)
    );

    // ALU
    Alu alu_unit (
        .A(alu_input1),
        .B(alu_input2),
        .ALU_Control( aluOp),
        .EN(1'b1),
        .Y(alu_result),
        .Zero(alu_zero),
        .Negative(alu_negative),
        .Positive(alu_positive)
    );

    // EX/MEM Pipeline Registers
    flipflop32 ex_mem_alu_result_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(alu_result),
        .out(ex_mem_alu_result),
        .reset(rst)
    );
    
    flipflop32 ex_mem_busB_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(id_ex_busB),
        .out(ex_mem_busB),
        .reset(rst)
    );

    // MEM/WB Pipeline Registers
    flipflop32 mem_wb_alu_result_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(ex_mem_alu_result),
        .out(mem_wb_alu_result),
        .reset(rst)
    );
    
    flipflop32 mem_wb_mem_data_reg (
        .clk(clk),
        .writeEn(1'b1),
        .in(mem_data_in),
        .out(mem_wb_mem_data),
        .reset(rst)
    );

    // Write Back Data Multiplexer
    mux432b wb_mux (
        .d0(mem_wb_alu_result),
        .d1(mem_wb_mem_data),
        .d2(pc_plus_one),
        .d3(extended_immediate),
        .s(wbData),
        .y(write_data)
    );

    assign pc_address = pc_current;
    assign mem_address = ex_mem_alu_result;
    assign mem_data_out = ex_mem_busB;							
	
    assign instruction_out = if_id_instruction;
    assign alu_result_out = alu_result;
    assign read_data1_out = read_data1;
    assign read_data2_out = read_data2;
    assign write_data_out = write_data;
    assign write_reg_out = write_reg;

endmodule	











			  

module datapath_tb;

    // Inputs
    reg clk;
    reg rst;
    reg [31:0] instruction_in;
    reg [31:0] mem_data_in;
    reg [5:0] opcode;
    reg [3:0] rd, rs, rt;
    reg [13:0] immediate;
    reg [1:0] pcSrc, regDest, readRd, aluSrc, wbData;
    reg extOp, regWr, disablePC;
    reg [3:0] aluOp;

    // Outputs
    wire [31:0] pc_address;
    wire [31:0] mem_address;
    wire [31:0] mem_data_out;
    wire alu_zero, alu_negative, alu_positive;
    wire [31:0] instruction_out;
    wire [31:0] alu_result_out;
    wire [31:0] read_data1_out;
    wire [31:0] read_data2_out;
    wire [31:0] write_data_out;
    wire [3:0] write_reg_out;

    // Instantiate the Unit Under Test (UUT)
    datapath uut (
        .clk(clk),
        .rst(rst),
        .instruction_in(instruction_in),
        .mem_data_in(mem_data_in),
        .opcode(opcode),
        .rd(rd), .rs(rs), .rt(rt),
        .immediate(immediate),
        .pcSrc(pcSrc), .regDest(regDest), .readRd(readRd),
        .aluSrc(aluSrc), .wbData(wbData),
        .extOp(extOp), .regWr(regWr), .disablePC(disablePC),
        .aluOp(aluOp),
        .pc_address(pc_address),
        .mem_address(mem_address),
        .mem_data_out(mem_data_out),
        .alu_zero(alu_zero), .alu_negative(alu_negative), .alu_positive(alu_positive),
        .instruction_out(instruction_out),
        .alu_result_out(alu_result_out),
        .read_data1_out(read_data1_out),
        .read_data2_out(read_data2_out),
        .write_data_out(write_data_out),
        .write_reg_out(write_reg_out)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        $display("Starting Datapath Testbench");
        $monitor("Time %0t | PC=%h, Instr=%h, ALU=%h, Read1=%h, Read2=%h, WriteData=%h, WRReg=%h",
                  $time, pc_address, instruction_out, alu_result_out, read_data1_out, read_data2_out, write_data_out, write_reg_out);

        // Initialize Inputs
        clk = 0;
        rst = 1;
        instruction_in = 32'h00000000;
        mem_data_in = 32'h00000000;
        opcode = 6'b000000;
        rd = 4'd1;
        rs = 4'd2;
        rt = 4'd3;
        immediate = 14'd5;
        pcSrc = 2'b00;
        regDest = 2'b00;
        readRd = 2'b00;
        aluSrc = 2'b00;
        wbData = 2'b00;
        extOp = 1'b0;
        regWr = 1'b0;
        disablePC = 1'b0;
        aluOp = 4'b0000;

        // Apply reset
        #10;
        rst = 0;

        // Set up test values for first instruction: 
        regDest = 2'b00;   // write to rd
        readRd = 2'b00;    // read from rt
        aluSrc = 2'b00;    // use reg
        wbData = 2'b00;    // write back ALU result
        regWr = 1'b1;      // enable register write
        extOp = 1'b0;
        aluOp = 4'b0000;   // ALU add
        rs = 4'd2;
        rt = 4'd3;
        rd = 4'd1;

        

        // Observe result: expect R1 = 25
        #20;

        // Add another test: ADDI R4 = R1 + 5
        rs = 4'd1;
        rt = 4'd0;
        rd = 4'd4;
        immediate = 14'd5;
        aluSrc = 2'b01;     // select immediate
        extOp = 1'b1;       // sign extend
        regDest = 2'b00;
        wbData = 2'b00;
        aluOp = 4'b0000;    // add
        regWr = 1'b1;

        #20;

        // Observe result: expect R4 = 30
        #20;

        // Finish simulation
        $finish;
    end

endmodule

		 