`timescale 1ns / 1ps

module computer (
    input wire CLK,
    input wire reset,
    output wire [2:0] current_state,
    output wire [2:0] cycle_count,
    
    // Original debug outputs from CPU
    output wire [31:0] instruction_out,
    output wire [31:0] alu_result_out,
    output wire [31:0] read_data1_out,
    output wire [31:0] read_data2_out,
    output wire [31:0] write_data_out,
    output wire [3:0] write_reg_out,
    
    // Essential additional debug outputs for testing
    output wire [31:0] pc_out,                // Program Counter
    output wire [31:0] mem_address_out,       // Memory address
    output wire [31:0] mem_data_out_debug,    // Memory data output
    output wire [31:0] mem_data_in_out,       // Memory data input
    output wire mem_read_out,                 // Memory read signal
    output wire mem_write_out                 // Memory write signal
);

    // Internal signals
    wire clk;
    wire [31:0] instruction_data;
    wire [31:0] mem_data_out;
    wire [31:0] mem_data_in;
    wire [31:0] mem_address;
    wire [31:0] pc_address;
    wire mem_read, mem_write, ldw_sdw, cycle_state;
    
    // Direct clock assignment
    assign clk = CLK;
    
    // Connect internal signals to debug outputs
    assign pc_out = pc_address;
    assign mem_address_out = mem_address;
    assign mem_data_out_debug = mem_data_out;
    assign mem_data_in_out = mem_data_in;
    assign mem_read_out = mem_read;
    assign mem_write_out = mem_write;
    
    // Instruction Memory
    instructionMemory imem (
        .clock(clk),
        .en(1'b1),
        .AddressBus(pc_address),
        .InstructionReg(instruction_data)
    );
    
    // Data Memory
    data_memory dmem (
        .clk(clk),
        .reset(reset),
        .memRd(mem_read),
        .memWr(mem_write),
        .address(mem_address),
        .data_in(mem_data_in),
        .EN(1'b1),
        .ldw_sdw(ldw_sdw),
        .second_cycle(cycle_state),
        .data_out(mem_data_out)
    );
    
    // CPU Core
    CPU cpu (
        .clk(clk),
        .rst(reset),
        .instruction_in(instruction_data),
        .mem_data_in(mem_data_out),
        .pc_address(pc_address),
        .mem_address(mem_address),
        .mem_data_out(mem_data_in),
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
    
endmodule


