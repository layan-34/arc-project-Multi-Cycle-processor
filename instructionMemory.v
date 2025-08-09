module instructionMemory(
    input wire clock,
    input wire en,
    input wire [31:0] AddressBus,    // Word-addressable (32-bit words)
    output reg [31:0] InstructionReg // 32-bit instruction output
);
    // 1024-word instruction memory (32-bit words)
    reg [31:0] instruction_memory [0:1023];
    
    // Initialize with test program
    initial begin
        // Clear memory
        for (integer i = 0; i < 1024; i = i + 1)
            instruction_memory[i] = 32'b0;
        											
       // OR R1, R2, R3       ; R1 = R2 | R3
instruction_memory[0] = 32'b000000_0001_0010_0011_00000000000000;

// ADD R4, R8, R9      ; R4 = R8 + R9
instruction_memory[1] = 32'b000001_0100_1000_1001_00000000000000;

// SUB R5, R6, R7      ; R5 = R6 - R7
instruction_memory[2] = 32'b000010_0101_0110_0111_00000000000000;

// CMP R10, R4, R5     ; R10 = compare R4 and R5
instruction_memory[3] = 32'b000011_1010_0100_0101_00000000000000;

// ORI R1, R2, 15      ; R1 = R2 | 15
instruction_memory[4] = 32'b000100_0001_0010_0000_00000000111111;

// ADDI R2, R2, -4     ; R2 = R2 - 4 (imm = 14-bit sign extended)
instruction_memory[5] = 32'b000101_0010_0010_0000_11111111111100;

// LW R3, 4(R1)        ; R3 = MEM[R1 + 4]
instruction_memory[6] = 32'b000110_0011_0001_0000_00000000000010;

// SW R3, 8(R1)        ; MEM[R1 + 8] = R3
instruction_memory[7] = 32'b000111_0011_0001_0000_00000000000100;



// LDW R2, 0(R1)       ; R0 = MEM[R1 + 0], R1 = MEM[R1 + 1]
instruction_memory[8] = 32'b001000_0000_0001_0000_00000000000000;

// SDW R3, 2(R4)       ; MEM[R3+2] = R2, MEM[R3+3] = R3
instruction_memory[9] = 32'b001001_0010_0011_0000_00000000000001;





// BZ R4, +2           ; if R4 == 0, skip next instr
instruction_memory[10] = 32'b001010_0000_0100_0000_00000000000001;

// ADDI R5, R5, 1      ; (may be skipped if R4 == 0)
instruction_memory[11] = 32'b000101_0101_0101_0000_00000000000001;

// J to instruction 0  ; jump to start
instruction_memory[12] = 32'b001110_0000_0000_0000_00000000000000;



// CALL +2             ; R14 = PC + 1, PC += 2
instruction_memory[13] = 32'b001111_0000_0000_0000_00000000000001;

// JR R3              ; jump back to return address
instruction_memory[14] = 32'b001101_0000_0111_0000_00000000000000;






       
    end
    
    // Read instruction from memory (synchronous read)
    always @(posedge clock) begin
        if (en) begin
            InstructionReg <= instruction_memory[AddressBus[11:2]]; // Word-aligned addressing
        end
    end
    
endmodule

// Test bench for verification
module tb_instructionMemory;
    reg clock;
    reg en;
    reg [31:0] AddressBus;
    wire [31:0] InstructionReg;
    
    // Instantiate the instruction memory
    instructionMemory uut (
        .clock(clock),
        .en(en),
        .AddressBus(AddressBus),
        .InstructionReg(InstructionReg)
    );
    
    // Clock generation
    initial begin
        clock = 0;
        forever #5 clock = ~clock;
    end
    
    // Test sequence
    initial begin
        $dumpfile("instruction_memory.vcd");
        $dumpvars(0, tb_instructionMemory);
        
        // Initialize
        en = 0;
        AddressBus = 0;
        
        #10;
        en = 1;
        
        // Test reading instructions sequentially
        for (integer i = 0; i < 12; i = i + 1) begin
            AddressBus = i * 4; // Word-aligned addresses (0, 4, 8, 12, ...)
            #10;
            $display("Address: %0d, Instruction: 0x%08h", AddressBus, InstructionReg);
        end
        
        #20;
        $finish;
    end
    
endmodule