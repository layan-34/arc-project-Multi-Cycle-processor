`timescale 1ns / 1ps


module CPU (
    input wire clk,
    input wire rst,
    input wire [31:0] instruction_in,
    input wire [31:0] mem_data_in,
    
	
	
	output wire [31:0] pc_address,
    output wire [31:0] mem_address,
    output wire [31:0] mem_data_out,
    output wire mem_read,
    output wire mem_write,
    output wire ldw_sdw,
    output wire cycle_state	,
	output wire [2:0] current_state,
output wire [2:0] cycle_count ,
 output wire [31:0] instruction_out,
    output wire [31:0] alu_result_out,
    output wire [31:0] read_data1_out,
    output wire [31:0] read_data2_out,
    output wire [31:0] write_data_out,
    output wire [3:0] write_reg_out
	

);

    // Instruction field extraction
    wire [5:0] opcode;
    wire [3:0] rd, rs, rt;
    wire [13:0] immediate;
    
    assign opcode = instruction_in[31:26];
    assign rd = instruction_in[25:22];
    assign rs = instruction_in[21:18];
    assign rt = instruction_in[17:14];
    assign immediate = instruction_in[13:0];

    // Control Unit signals
    wire [1:0] pcSrc, regDest, readRd, aluSrc, wbData;
    wire extOp, regWr, memRead, memWr, exception;
    wire [3:0] aluOp;
    wire disablePC;
    
	
	
	
	wire if_id_enable, id_ex_enable, ex_mem_enable, mem_wb_enable;
    
	
	
    // ALU flags
    wire alu_zero, alu_negative, alu_positive;
    
    // Multi-cycle control
    reg cycle_state_reg;
    
    // Multi-cycle state management
    always @(posedge clk) begin
        if (rst) begin
            cycle_state_reg <= 1'b0;
        end else if (ldw_sdw && !cycle_state_reg) begin
            cycle_state_reg <= 1'b1;
        end else if (ldw_sdw && cycle_state_reg) begin
            cycle_state_reg <= 1'b0;
        end else begin
            cycle_state_reg <= 1'b0;
        end
    end
    
    assign cycle_state = cycle_state_reg;

 datapath dp (
        .clk(clk),
        .rst(rst),
        .instruction_in(instruction_in),
        .mem_data_in(mem_data_in),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .immediate(immediate),
        .pcSrc(pcSrc),
        .regDest(regDest),
        .readRd(readRd),
        .extOp(extOp),
        .regWr(regWr),
        .aluSrc(aluSrc),
        .aluOp(aluOp),
        .wbData(wbData),
        .disablePC(disablePC),
        .pc_address(pc_address),
        .mem_address(mem_address),
        .mem_data_out(mem_data_out),
        .alu_zero(alu_zero),
        .alu_negative(alu_negative),
        .alu_positive(alu_positive),
        .instruction_out(instruction_out),
        .alu_result_out(alu_result_out),
        .read_data1_out(read_data1_out),
        .read_data2_out(read_data2_out),
        .write_data_out(write_data_out),
        .write_reg_out(write_reg_out)
    );

  control_unit ctrl_unit (
    .clk(clk),
    .rst(rst),
    .opcode(opcode),
    .rd(rd),
    .rs(rs),
    .negative(alu_negative),
    .positive(alu_positive),
    .zero(alu_zero),
    .pcSrc(pcSrc),
    .regDest(regDest),
    .readRd(readRd),
    .extOp(extOp),
    .regWr(regWr),
    .aluSrc(aluSrc),
    .aluOp(aluOp),
    .memRead(memRead),
    .memWr(memWr),
    .wbData(wbData),
    .ldw_sdw(ldw_sdw),
    .exception(exception),
    .disablePC(disablePC),
    .if_id_enable(if_id_enable),
        .id_ex_enable(id_ex_enable),
        .ex_mem_enable(ex_mem_enable),
        .mem_wb_enable(mem_wb_enable),
        .current_state(current_state),
    .cycle_count(cycle_count)
);

    
    // Connect memory control signals
    assign mem_read = memRead;
    assign mem_write = memWr;

endmodule 











	 `timescale 1ns / 1ps

module CPU_tb;

    // Inputs
    reg clk;
    reg rst;
    reg [31:0] instruction_in;
    reg [31:0] mem_data_in;

    // Outputs
    wire [31:0] pc_address;
    wire [31:0] mem_address;
    wire [31:0] mem_data_out;
    wire mem_read;
    wire mem_write;
    wire ldw_sdw;
    wire cycle_state;
    wire [2:0] current_state;
    wire [2:0] cycle_count;
    wire [31:0] instruction_out;
    wire [31:0] alu_result_out;
    wire [31:0] read_data1_out;
    wire [31:0] read_data2_out;
    wire [31:0] write_data_out;
    wire [3:0] write_reg_out;

    // Instantiate the CPU
    CPU uut (
        .clk(clk),
        .rst(rst),
        .instruction_in(instruction_in),
        .mem_data_in(mem_data_in),
        .pc_address(pc_address),
        .mem_address(mem_address),
        .mem_data_out(mem_data_out),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .ldw_sdw(ldw_sdw),
        .cycle_state(cycle_state),
        .current_state(current_state),
        .cycle_count(cycle_count),
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
        // Initial values
        clk = 0;
        rst = 1;
        instruction_in = 0;
        mem_data_in = 32'h00000000;

        // Hold reset for 2 cycles
        #10;
        rst = 0;

        // --- Test 1: Add R2 = R0 + R1
        // opcode = 6'b000001, rd=2, rs=0, rt=1, immediate=ignored
        instruction_in = {6'b000001, 4'd2, 4'd0, 4'd1, 14'd0};
        #10;

        // --- Test 2: ADDI R3 = R1 + 5
        // opcode = 6'b000010, rd=3, rs=1, rt=ignored, immediate=5
        instruction_in = {6'b000010, 4'd3, 4'd1, 4'd0, 14'd5};
        #10;

        // --- Test 3: Load Word LW R4 = MEM[R2 + 1]
        // opcode = 6'b000011, rd=4, rs=2, rt=ignored, imm = 1
        instruction_in = {6'b000011, 4'd4, 4'd2, 4'd0, 14'd1};
        mem_data_in = 32'hDEADBEEF; // data from memory
        #20;

        // --- Test 4: Store Word SW MEM[R2 + 2] = R4
        // opcode = 6'b000100, rd=4, rs=2, rt=ignored, imm = 2
        instruction_in = {6'b000100, 4'd4, 4'd2, 4'd0, 14'd2};
        #20;

        // --- Test 5: Branch (dummy)
        instruction_in = {6'b000101, 4'd0, 4'd1, 4'd2, 14'd4};
        #10;

        // End of simulation
        $finish;
    end

endmodule
