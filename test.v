`timescale 1ns / 1ps
module test22();
    
    // Inputs
    reg CLK;
    reg reset;
    
    // Outputs
    wire [2:0] current_state;
    wire [2:0] cycle_count;
    wire [31:0] instruction_out;
    wire [31:0] alu_result_out;
    wire [31:0] read_data1_out;
    wire [31:0] read_data2_out;
    wire [31:0] write_data_out;
    wire [3:0] write_reg_out;
    wire [31:0] pc_out;
    wire [31:0] mem_address_out;
    wire [31:0] mem_data_out_debug;
    wire [31:0] mem_data_in_out;
    wire mem_read_out;
    wire mem_write_out;
    
    // Instantiate the Unit Under Test (UUT)
    computer uut (
        .CLK(CLK),
        .reset(reset),
        .current_state(current_state),
        .cycle_count(cycle_count),
        .instruction_out(instruction_out),
        .alu_result_out(alu_result_out),
        .read_data1_out(read_data1_out),
        .read_data2_out(read_data2_out),
        .write_data_out(write_data_out),
        .write_reg_out(write_reg_out),
        .pc_out(pc_out),
        .mem_address_out(mem_address_out),
        .mem_data_out_debug(mem_data_out_debug),
        .mem_data_in_out(mem_data_in_out),
        .mem_read_out(mem_read_out),
        .mem_write_out(mem_write_out)
    );
    
    // Clock generation - 10ns period (100MHz)
    initial begin
        CLK = 0;
        forever #5 CLK = ~CLK;
    end
    
    // Test stimulus
    initial begin
        // Initialize Inputs
        reset = 1;
        
        // Wait for global reset
        #1;
        reset = 0;
        
        // Display header
        $display("Time\t| PC\t| Instruction\t| State | Cycle | ALU Result\t| Reg1\t| Reg2\t| WriteData\t| WReg | MemAddr\t| MemRead | MemWrite");
        $display("-------|-------|---------------|-------|-------|-----------|-------|-------|-----------|------|-----------|---------|----------");
        
        // Monitor signals every clock cycle
        forever begin
            @(posedge CLK);
            $display("%0t\t| %h\t| %h\t| %d\t| %d\t| %h\t| %h\t| %h\t| %h\t| %d\t| %h\t| %b\t| %b",
                $time, pc_out, instruction_out, current_state, cycle_count, 
                alu_result_out, read_data1_out, read_data2_out, write_data_out, 
                write_reg_out, mem_address_out, mem_read_out, mem_write_out);
        end
    end
    
    // Additional monitoring for memory operations
    always @(posedge CLK) begin
        if (mem_read_out) begin
            $display("*** MEMORY READ: Address=%h, Data=%h at time %0t", 
                     mem_address_out, mem_data_out_debug, $time);
        end
        if (mem_write_out) begin
            $display("*** MEMORY WRITE: Address=%h, Data=%h at time %0t", 
                     mem_address_out, mem_data_in_out, $time);
        end
    end
    
    // Test timeout and finish
    initial begin
        #10000; // Run for 10us
        $display("\n=== Test completed at time %0t ===", $time);
        $finish;
    end
    
   
    
    // Optional: Reset pulse during simulation
    initial begin
        #1000;  // After 1us
        $display("\n*** Applying reset pulse at time %0t ***", $time);
        reset = 1;
        #20;
        reset = 0;
        $display("*** Reset released at time %0t ***\n", $time);
    end
    
endmodule