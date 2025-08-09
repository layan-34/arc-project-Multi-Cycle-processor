
module data_memory(
    input clk,
    input reset,
    input memRd,           // Memory read enable
    input memWr,           // Memory write enable
    input [31:0] address,  // Word address (32-bit word addressable)
    input [31:0] data_in,  // 32-bit data to write
    input EN,              // Enable signal
    input ldw_sdw,         // Signal for LDW/SDW operations
    input second_cycle,    // Signal indicating second cycle of LDW/SDW
    output reg [31:0] data_out   // 32-bit data read from memory
);

    // Memory organized as 32-bit words (word addressable)
    // Each address directly contains a complete 32-bit word
    reg [31:0] DATA_RAM[1023:0];  // 1024 words × 32 bits = 4KB total
    
    // Initialize data memory with some test data
    initial begin
        // Initialize all memory to zero
        integer i;
        for (i = 0; i < 1024; i = i + 1) begin
            DATA_RAM[i] = 32'h00000000;
        end
        
        // Add some test data (32-bit values at word addresses)
    DATA_RAM[2] = 32'hDEADBEEF;  
    DATA_RAM[3] = 32'hCAFEBABE;  
    DATA_RAM[4] = 32'h12345678;  
    DATA_RAM[5] = 32'h87654321;  
	
    end
    
    // Memory operations
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            data_out <= 32'h00000000;
        end else if (EN) begin
            // Memory Write Operations
            if (memWr) begin
                if (ldw_sdw) begin
                    // SDW operation - store double word (two 32-bit words)
                    if (!second_cycle) begin
                        // First cycle: store first 32-bit word at address
                        DATA_RAM[address] <= data_in;
                    end else begin
                        // Second cycle: store second 32-bit word at address+1
                        DATA_RAM[address + 1] <= data_in;
                    end
                end else begin
                    // Regular SW operation - store single 32-bit word
                    DATA_RAM[address] <= data_in;
                end
            end
            
            // Memory Read Operations
            if (memRd) begin
                if (ldw_sdw) begin
                    // LDW operation - load double word
                    if (!second_cycle) begin
                        // First cycle: load first 32-bit word from address
                        data_out <= DATA_RAM[address];
                    end else begin
                        // Second cycle: load second 32-bit word from address+1
                        data_out <= DATA_RAM[address + 1];
                    end
                end else begin
                    // Regular LW operation - load single 32-bit word
                    data_out <= DATA_RAM[address];
                end
            end
        end
    end

endmodule		





`timescale 1ns / 1ps
















`timescale 1ns / 1ps

module data_memory_tb;

    // Inputs
    reg clk;
    reg reset;
    reg memRd;
    reg memWr;
    reg [31:0] address;
    reg [31:0] data_in;
    reg EN;
    reg ldw_sdw;
    reg second_cycle;

    // Output
    wire [31:0] data_out;

    // Instantiate the Unit Under Test (UUT)
    data_memory uut (
        .clk(clk),
        .reset(reset),
        .memRd(memRd),
        .memWr(memWr),
        .address(address),
        .data_in(data_in),
        .EN(EN),
        .ldw_sdw(ldw_sdw),
        .second_cycle(second_cycle),
        .data_out(data_out)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize inputs
        clk = 0;
        reset = 1;
        memRd = 0;
        memWr = 0;
        address = 0;
        data_in = 0;
        EN = 0;
        ldw_sdw = 0;
        second_cycle = 0;

        $display("\n=== Starting Testbench ===\n");
        $display("Time\tOp\tAddr\tSecCycle\tDataIn\t\tDataOut");

        // Apply reset
        #10 reset = 0;
        EN = 1;

        // === Single Word Write ===
        write_word(10, 32'hABCD1234);
        read_word(10);

        // === Double Word Write (SDW) ===
        write_double_word(20, 32'h11112222, 32'h33334444);
        read_double_word(20);

        // === Read Initialized Memory (from module) ===
        read_word(100);  // Expect DEADBEEF
        read_word(200);  // Expect CAFEBABE
        read_word(300);  // Expect 12345678
        read_word(400);  // Expect 87654321

        // === Disable memory and try write (should not affect memory) ===
        EN = 0;
        write_word(50, 32'hBADBAD00);
        EN = 1;
        read_word(50);   // Should still be 0

        $display("\n=== Testbench Completed ===");
        $finish;
    end

    // Tasks

    // Write a single word
    task write_word(input [31:0] addr, input [31:0] data);
        begin
            #10;
            address = addr;
            data_in = data;
            memWr = 1;
            ldw_sdw = 0;
            second_cycle = 0;
            #10;
            memWr = 0;
        end
    endtask

    // Read a single word
    task read_word(input [31:0] addr);
        begin
            #10;
            address = addr;
            memRd = 1;
            ldw_sdw = 0;
            second_cycle = 0;
            #10;
            memRd = 0;
        end
    endtask

    // Write two words (SDW)
    task write_double_word(input [31:0] addr, input [31:0] data1, input [31:0] data2);
        begin
            // First cycle
            #10;
            address = addr;
            data_in = data1;
            memWr = 1;
            ldw_sdw = 1;
            second_cycle = 0;
            #10;

            // Second cycle
            data_in = data2;
            second_cycle = 1;
            #10;
            memWr = 0;
            ldw_sdw = 0;
            second_cycle = 0;
        end
    endtask

    // Read two words (LDW)
    task read_double_word(input [31:0] addr);
        begin
            // First cycle
            #10;
            address = addr;
            memRd = 1;
            ldw_sdw = 1;
            second_cycle = 0;
            #10;

            // Second cycle
            second_cycle = 1;
            #10;
            memRd = 0;
            ldw_sdw = 0;
            second_cycle = 0;
        end
    endtask

    // Monitor all activity
    always @(posedge clk) begin
        $display("%0t\t%s\t%d\t%b\t\t%h\t%h",
                 $time,
                 memWr ? "WR " : (memRd ? "RD " : "NOP"),
                 address,
                 second_cycle,
                 data_in,
                 data_out);
    end

endmodule












module data_memoryread_tb;

    // Inputs
    reg clk;
    reg reset;
    reg memRd;
    reg memWr;
    reg [31:0] address;
    reg [31:0] data_in;
    reg EN;
    reg ldw_sdw;
    reg second_cycle;

    // Output
    wire [31:0] data_out;

    // Instantiate the Unit Under Test (UUT)
    data_memory uut (
        .clk(clk),
        .reset(reset),
        .memRd(memRd),
        .memWr(memWr),
        .address(address),
        .data_in(data_in),
        .EN(EN),
        .ldw_sdw(ldw_sdw),
        .second_cycle(second_cycle),
        .data_out(data_out)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize inputs
        clk = 0;
        reset = 1;
        memRd = 0;
        memWr = 0;
        address = 0;
        data_in = 0;
        EN = 0;
        ldw_sdw = 0;
        second_cycle = 0;

        // Display header
        $display("Time\tEN\tOp\tAddr\tSecCycle\tDataIn\t\tDataOut");

        // Deassert reset
        #10 reset = 0;

        // Enable memory
        EN = 1;

        // ================================
        // Read initialized memory values
        // ================================
        read_and_check(0);
        read_and_check(1);
        read_and_check(2);
        read_and_check(3);
        read_and_check(10);
        read_and_check(11);
        read_and_check(12);
        read_and_check(13);
        read_and_check(14);
        read_and_check(15);

        // End test
        $display("\n--- Testbench completed ---");
        $finish;
    end

    // Helper task to read and display memory content
    task read_and_check(input [31:0] addr);
        begin
            #10 memRd = 1;
            address = addr;
            #10 memRd = 0;
        end
    endtask

    // Print trace on every positive edge of the clock
    always @(posedge clk) begin
        $display("%0t\t%b\t%s\t%d\t%b\t\t%h\t%h",
                 $time,
                 EN,
                 memRd ? "RD " : (memWr ? "WR " : "NOP"),
                 address,
                 second_cycle,
                 data_in,
                 data_out);
    end

endmodule






`timescale 1ns / 1ps

module data_memory_write_tb;

    // Inputs
    reg clk;
    reg reset;
    reg memRd;
    reg memWr;
    reg [31:0] address;
    reg [31:0] data_in;
    reg EN;
    reg ldw_sdw;
    reg second_cycle;

    // Output
    wire [31:0] data_out;

    // Instantiate the Unit Under Test (UUT)
    data_memory uut (
        .clk(clk),
        .reset(reset),
        .memRd(memRd),
        .memWr(memWr),
        .address(address),
        .data_in(data_in),
        .EN(EN),
        .ldw_sdw(ldw_sdw),
        .second_cycle(second_cycle),
        .data_out(data_out)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        EN = 0;
        memRd = 0;
        memWr = 0;
        address = 0;
        data_in = 0;
        ldw_sdw = 0;
        second_cycle = 0;

        // Release reset
        #10 reset = 0;
        EN = 1;

        $display("Time\tOp\tAddr\tSecCycle\tDataIn\t\tDataOut");

        // ======================
        // Write: Single Word
        // ======================
        write_word(20, 32'hA1B2C3D4);  // Address 20 <- A1B2C3D4
        read_word(20);

        // ======================
        // Write: Double Word (SDW)
        // ======================
        write_double_word(30, 32'h11112222, 32'h33334444);  // Address 30, 31
        read_word(30);
        read_word(31);

        $display("\n--- Write Testbench Completed ---");
        $finish;
    end

    // Tasks

    // Task to perform a single word write
    task write_word(input [31:0] addr, input [31:0] data);
        begin
            #10;
            address = addr;
            data_in = data;
            memWr = 1;
            ldw_sdw = 0;
            second_cycle = 0;
            #10 memWr = 0; // Deassert after write
        end
    endtask

    // Task to perform a double word write (SDW)
    task write_double_word(input [31:0] addr, input [31:0] data1, input [31:0] data2);
        begin
            // First cycle
            #10;
            address = addr;
            data_in = data1;
            memWr = 1;
            ldw_sdw = 1;
            second_cycle = 0;
            #10;

            // Second cycle
            data_in = data2;
            second_cycle = 1;
            #10 memWr = 0; // Deassert
            ldw_sdw = 0;
            second_cycle = 0;
        end
    endtask

    // Task to read and display word
    task read_word(input [31:0] addr);
        begin
            #10;
            address = addr;
            memRd = 1;
            #10 memRd = 0;
        end
    endtask

    // Trace on every posedge clk
    always @(posedge clk) begin
        $display("%0t\t%s\t%d\t%b\t\t%h\t%h",
                 $time,
                 memWr ? "WR " : (memRd ? "RD " : "NOP"),
                 address,
                 second_cycle,
                 data_in,
                 data_out);
    end

endmodule


