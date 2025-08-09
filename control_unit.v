module control_unit (
    input wire clk,
    input wire rst,
    input wire [5:0] opcode,
    input wire [3:0] rd,            // Destination register for LDW exception check
    input wire [3:0] rs,            // Source register for SDW exception check
    input wire negative,            // ALU negative flag
    input wire positive,            // ALU positive flag  
    input wire zero,                // ALU zero flag
    
    // Control outputs
    output reg [1:0] pcSrc,
    output reg [1:0] regDest,
    output reg [1:0] readRd,
    output reg extOp,
    output reg regWr,
    output reg [1:0] aluSrc,
    output reg [3:0] aluOp,
    output reg memRead,
    output reg memWr,
    output reg [1:0] wbData,
    output reg ldw_sdw,
    output reg exception,
    output reg disablePC,
    output reg if_id_enable,
output reg id_ex_enable, 
output reg ex_mem_enable,
output reg mem_wb_enable,

    // State outputs for debugging
    output reg [2:0] current_state,
    output reg [2:0] cycle_count
);

// State definitions
parameter [2:0] FETCH     = 3'b000,
                DECODE    = 3'b001, 
                EXECUTE   = 3'b010,
                MEMORY    = 3'b011,
                WRITEBACK = 3'b100,
                EXCEPTION_STATE = 3'b101;

// Internal state registers
reg [2:0] state, next_state;
reg [2:0] cycles, next_cycles;
reg [5:0] stored_opcode;
reg [3:0] stored_rd, stored_rs;

// Instruction type detection
wire is_alu_reg, is_alu_imm, is_load, is_store, is_branch, is_jump, is_ldw, is_sdw;
wire is_multi_cycle;

assign is_alu_reg = (opcode >= 6'b000000) && (opcode <= 6'b000011); // OR, ADD, SUB, CMP
assign is_alu_imm = (opcode == 6'b000100) || (opcode == 6'b000101); // ORI, ADDI
assign is_load    = (opcode == 6'b000110); // LW
assign is_store   = (opcode == 6'b000111); // SW
assign is_ldw     = (opcode == 6'b001000); // LDW
assign is_sdw     = (opcode == 6'b001001); // SDW
assign is_branch  = (opcode >= 6'b001010) && (opcode <= 6'b001100); // BZ, BGZ, BLZ
assign is_jump    = (opcode >= 6'b001101) && (opcode <= 6'b001111); // JR, J, CALL

assign is_multi_cycle = is_ldw || is_sdw;

// Exception detection
wire ldw_exception, sdw_exception, invalid_opcode_exception;
assign ldw_exception = is_ldw && rd[0];
assign sdw_exception = is_sdw && rs[0];
assign invalid_opcode_exception = (opcode > 6'b001111);
wire any_exception = ldw_exception || sdw_exception || invalid_opcode_exception;



 // Use stored opcode for multi-cycle operations
 wire [5:0] current_opcode = is_multi_cycle ? stored_opcode : opcode;
 wire [3:0] current_rd = is_multi_cycle ? stored_rd : rd;
            
// Modified control logic in your control unit
always @(*) begin
    // Default - disable pipeline advancement
    if_id_enable = 1'b0;
    id_ex_enable = 1'b0;
    ex_mem_enable = 1'b0;
    mem_wb_enable = 1'b0;
    
    case (state)
        FETCH: begin
            if_id_enable = 1'b1;  // Allow IF/ID to advance
        end
        
        DECODE: begin
            id_ex_enable = 1'b1;  // Allow ID/EX to advance
        end
        
        EXECUTE: begin
            ex_mem_enable = 1'b1; // Allow EX/MEM to advance
        end
        
        MEMORY: begin
            mem_wb_enable = 1'b1; // Allow MEM/WB to advance
        end
        
        WRITEBACK: begin
            // Pipeline can advance to next instruction
            if_id_enable = 1'b1;
        end
    endcase
end




// State machine
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state <= FETCH;
        cycles <= 3'b000;
        stored_opcode <= 6'b000000;
        stored_rd <= 4'b0000;
        stored_rs <= 4'b0000;
    end else begin
        state <= next_state;
        cycles <= next_cycles;
        
        // Store instruction info during DECODE for multi-cycle operations
        if (state == DECODE) begin
            stored_opcode <= opcode;
            stored_rd <= rd;
            stored_rs <= rs;
        end
    end
end

// Next state logic
always @(*) begin
    next_state = state;
    next_cycles = cycles;
    
    case (state)
        FETCH: begin
            next_state = DECODE;
            next_cycles = 3'b000;
        end
        
        DECODE: begin
            if (any_exception) begin
                next_state = EXCEPTION_STATE;
            end else begin
                next_state = EXECUTE;
            end
        end
        
        EXECUTE: begin
            if (is_load || is_store || is_ldw || is_sdw) begin
                next_state = MEMORY;
            end else if (is_alu_reg || is_alu_imm) begin
                next_state = WRITEBACK;
            end else begin // branches and jumps
                next_state = FETCH;
            end
        end
        
        MEMORY: begin
            if (is_multi_cycle) begin
                if (cycles == 3'b000) begin
                    // First cycle of LDW/SDW completed, do second cycle
                    next_cycles = 3'b001;
                    next_state = MEMORY; // Stay in MEMORY for second cycle
                end else begin
                    // Second cycle completed
                    next_state = WRITEBACK;
                end
            end else begin
                // Single cycle memory operation
                if (is_load) begin
                    next_state = WRITEBACK;
                end else begin // is_store
                    next_state = FETCH;
                end
            end
        end
        
        WRITEBACK: begin
            next_state = FETCH;
        end
        
        EXCEPTION_STATE: begin
            next_state = EXCEPTION_STATE; // Stay in exception until reset
        end
        
        default: begin
            next_state = FETCH;
        end
    endcase
end

// Control signal generation based on current state
always @(*) begin
    // Default values
    pcSrc = 2'b00;
    regDest = 2'b00;
    readRd = 2'b00;
    extOp = 1'b0;
    regWr = 1'b0;
    aluSrc = 2'b00;
    aluOp = 4'b0000;
    memRead = 1'b0;
    memWr = 1'b0;
    wbData = 2'b00;
    ldw_sdw = 1'b0;
    exception = 1'b0;
    disablePC = 1'b0;
    current_state = state;
    cycle_count = cycles;
    
    case (state)
        FETCH: begin
            // PC increment happens automatically
            // Instruction memory read enabled by default
        end
        
        DECODE: begin
            disablePC = 1'b1; // Disable PC increment during decode
            // Decode logic happens here
        end
        
        EXECUTE: begin
            disablePC = 1'b1;
            
            // Set ALU operation
            aluOp = opcode[3:0];
            
            // Set ALU source
            if (is_alu_imm || is_load || is_store || is_ldw || is_sdw) begin
                aluSrc = 2'b01; // Immediate
                extOp = 1'b1;   // Sign extend for addresses
            end else begin
                aluSrc = 2'b00; // Register
            end
            
            // Special handling for logical operations (zero extend)
            if (opcode == 6'b000000 || opcode == 6'b000100) begin // OR, ORI
                extOp = 1'b0; // Zero extend
            end
            
            // Set readRd for store operations
            if (is_store || is_sdw) begin
                readRd = 2'b01; // Read Rd for store data
            end
        end
        
        MEMORY: begin
            disablePC = 1'b1;
            
           
            if (current_opcode == 6'b000110 || current_opcode == 6'b001000) begin // LW, LDW
                memRead = 1'b1;
                
                if (current_opcode == 6'b001000 && cycles == 3'b001) begin // LDW second cycle
                    regDest = 2'b01; // Rd+1
                    aluSrc = 2'b10;  // Address+1
                end
            end
            
            if (current_opcode == 6'b000111 || current_opcode == 6'b001001) begin // SW, SDW
                memWr = 1'b1;
                readRd = 2'b01; // Read data to store
                
                if (current_opcode == 6'b001001 && cycles == 3'b001) begin // SDW second cycle
                    readRd = 2'b10; // Rd+1
                    aluSrc = 2'b10; // Address+1
                end
            end
            
            ldw_sdw = (current_opcode == 6'b001000 || current_opcode == 6'b001001);
        end
        
        WRITEBACK: begin
            disablePC = 1'b1;
            
          
            regWr = 1'b1;
            
            // Set write destination
            if (current_opcode == 6'b001000 && cycles == 3'b001) begin // LDW second cycle
                regDest = 2'b01; // Rd+1
            end else if (current_opcode == 6'b001111) begin // CALL
                regDest = 2'b10; // R14
            end
            
            // Set write data source
            if (current_opcode == 6'b000110 || current_opcode == 6'b001000) begin // LW, LDW
                wbData = 2'b01; // Memory data
            end else if (current_opcode == 6'b001111) begin // CALL
                wbData = 2'b10; // PC value
            end else begin
                wbData = 2'b00; // ALU result
            end
            
            // Handle branches and jumps (set PC source)
            if (opcode == 6'b001010 && zero) begin // BZ taken
                pcSrc = 2'b01;
            end else if (opcode == 6'b001011 && positive && !zero) begin // BGZ taken
                pcSrc = 2'b01;
            end else if (opcode == 6'b001100 && negative && !zero) begin // BLZ taken
                pcSrc = 2'b01;
            end else if (opcode >= 6'b001101) begin // JR, J, CALL
                pcSrc = 2'b10;
            end
        end
        
        EXCEPTION_STATE: begin
            exception = 1'b1;
            disablePC = 1'b1;
        end
    endcase
end	  
					 

endmodule	  






















 `timescale 1ns / 1ps

module control_unit_tb();

    // Inputs
    reg clk;
    reg rst;
    reg [5:0] opcode;
    reg [3:0] rd;
    reg [3:0] rs;
    reg negative;
    reg positive;
    reg zero;
    
    // Outputs
    wire [1:0] pcSrc;
    wire [1:0] regDest;
    wire [1:0] readRd;
    wire extOp;
    wire regWr;
    wire [1:0] aluSrc;
    wire [3:0] aluOp;
    wire memRead;
    wire memWr;
    wire [1:0] wbData;
    wire ldw_sdw;
    wire exception;
    wire disablePC;
    wire if_id_enable;
    wire id_ex_enable;
    wire ex_mem_enable;
    wire mem_wb_enable;
    wire [2:0] current_state;
    wire [2:0] cycle_count;
    
    // Instantiate the Unit Under Test (UUT)
    control_unit uut (
        .clk(clk),
        .rst(rst),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .negative(negative),
        .positive(positive),
        .zero(zero),
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
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test procedure
    initial begin
        // Initialize Inputs
        rst = 1;
        opcode = 6'b000000;
        rd = 4'b0000;
        rs = 4'b0000;
        negative = 0;
        positive = 0;
        zero = 0;
        
        // Wait for global reset
        #20;
        rst = 0;
        
        // Test 1: ALU Register Operation (ADD)
        $display("Test 1: ALU Register Operation (ADD)");
        opcode = 6'b000001; // ADD
        rd = 4'b0010;
        rs = 4'b0011;
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b100); // EXECUTE to WRITEBACK
        #10;
        verify_state_transition(3'b100, 3'b000); // WRITEBACK to FETCH
        
        // Test 2: ALU Immediate Operation (ADDI)
        $display("\nTest 2: ALU Immediate Operation (ADDI)");
        opcode = 6'b000101; // ADDI
        rd = 4'b0100;
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b100); // EXECUTE to WRITEBACK
        #10;
        verify_state_transition(3'b100, 3'b000); // WRITEBACK to FETCH
        
        // Test 3: Load Word (LW)
        $display("\nTest 3: Load Word (LW)");
        opcode = 6'b000110; // LW
        rd = 4'b0101;
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b011); // EXECUTE to MEMORY
        #10;
        verify_state_transition(3'b011, 3'b100); // MEMORY to WRITEBACK
        #10;
        verify_state_transition(3'b100, 3'b000); // WRITEBACK to FETCH
        
        // Test 4: Store Word (SW)
        $display("\nTest 4: Store Word (SW)");
        opcode = 6'b000111; // SW
        rs = 4'b0110;
        rd = 4'b0111;
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b011); // EXECUTE to MEMORY
        #10;
        verify_state_transition(3'b011, 3'b000); // MEMORY to FETCH
        
        // Test 5: Load Double Word (LDW)
        $display("\nTest 5: Load Double Word (LDW)");
        opcode = 6'b001000; // LDW
        rd = 4'b1000; // Even register
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b011); // EXECUTE to MEMORY
        #10;
        // First cycle of LDW
        verify_cycle_count(3'b000);
        #10;
        // Second cycle of LDW
        verify_cycle_count(3'b001);
        verify_state_transition(3'b011, 3'b100); // MEMORY to WRITEBACK
        #10;
        verify_state_transition(3'b100, 3'b000); // WRITEBACK to FETCH
        
        // Test 6: Store Double Word (SDW)
        $display("\nTest 6: Store Double Word (SDW)");
        opcode = 6'b001001; // SDW
        rs = 4'b1010; // Even register
        rd = 4'b1011;
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b011); // EXECUTE to MEMORY
        #10;
        // First cycle of SDW
        verify_cycle_count(3'b000);
        #10;
        // Second cycle of SDW
        verify_cycle_count(3'b001);
        verify_state_transition(3'b011, 3'b000); // MEMORY to FETCH
        
        // Test 7: Branch (BZ taken)
        $display("\nTest 7: Branch (BZ taken)");
        opcode = 6'b001010; // BZ
        zero = 1;
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b000); // EXECUTE to FETCH (branch taken)
        zero = 0;
        
        // Test 8: Jump (J)
        $display("\nTest 8: Jump (J)");
        opcode = 6'b001110; // J
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b010); // DECODE to EXECUTE
        #10;
        verify_state_transition(3'b010, 3'b000); // EXECUTE to FETCH
        
        // Test 9: Exception (LDW with odd register)
        $display("\nTest 9: Exception (LDW with odd register)");
        opcode = 6'b001000; // LDW
        rd = 4'b1001; // Odd register - should cause exception
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b101); // DECODE to EXCEPTION_STATE
        #10;
        verify_state_transition(3'b101, 3'b101); // Should stay in exception state
        #10;
        
        // Reset and test invalid opcode exception
        $display("\nTest 10: Invalid Opcode Exception");
        rst = 1;
        #10;
        rst = 0;
        opcode = 6'b010000; // Invalid opcode
        #10;
        verify_state_transition(3'b000, 3'b001); // FETCH to DECODE
        #10;
        verify_state_transition(3'b001, 3'b101); // DECODE to EXCEPTION_STATE
        #10;
        
        $display("\nAll tests completed");
        $finish;
    end
    
    // Helper task to verify state transitions
    task verify_state_transition;
        input [2:0] expected_current;
        input [2:0] expected_next;
        begin
            if (current_state !== expected_current) begin
                $display("ERROR: At time %0t, current_state is %b, expected %b", 
                         $time, current_state, expected_current);
            end
            
            #1; // Small delay to allow for state transition
            
            if (uut.state !== expected_next) begin
                $display("ERROR: At time %0t, next_state is %b, expected %b", 
                         $time, uut.state, expected_next);
            end
        end
    endtask
    
    // Helper task to verify cycle count
    task verify_cycle_count;
        input [2:0] expected_cycles;
        begin
            if (cycle_count !== expected_cycles) begin
                $display("ERROR: At time %0t, cycle_count is %b, expected %b", 
                         $time, cycle_count, expected_cycles);
            end
        end
    endtask
    
    // Monitor important signals
    initial begin
        $monitor("Time=%0t State=%b Opcode=%b PC_DISABLE=%b EXCEPTION=%b", 
                 $time, current_state, opcode, disablePC, exception);
    end
    
endmodule
