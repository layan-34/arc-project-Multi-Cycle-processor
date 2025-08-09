		module registerFile(
    input wire clock,
    input wire RegWr, 
    input wire [3:0] RA, RB, RW,
    input wire [31:0] BusW,
    output reg [31:0] BusA, BusB
);

    reg [31:0] registers_array [15:0];
    
    always @(posedge clock) begin
        BusA <= registers_array[RA];
        BusB <= registers_array[RB];
    end

    always @(posedge clock) begin
        if (RegWr && RW != 3'b000) begin
            registers_array[RW] <= BusW;
        end
    end

    initial begin
        registers_array[0] <= 32'h00000000; // R0
        registers_array[1] <= 32'h00000001; // R1
        registers_array[2] <= 32'h00000002; // R2
        registers_array[3] <= 32'h00000003; // R3
        registers_array[4] <= 32'h00000004; // R4
        registers_array[5] <= 32'h00000005; // R5
        registers_array[6] <= 32'h00000006; // R6
        registers_array[7] <= 32'h00000007; // R7
		registers_array[8] <= 32'h00000008; // R8
		registers_array[9] <= 32'h00000009; // R9
		registers_array[10] <=32'h00000010; // R10
    end

endmodule	



 `timescale 1ns/1ps

module registerFile_tb;

    // Inputs
    reg clock;
    reg RegWr;
    reg [3:0] RA, RB, RW;
    reg [31:0] BusW;

    // Outputs
    wire [31:0] BusA, BusB;

    // Instantiate the Unit Under Test (UUT)
    registerFile uut (
        .clock(clock),
        .RegWr(RegWr),
        .RA(RA),
        .RB(RB),
        .RW(RW),
        .BusW(BusW),
        .BusA(BusA),
        .BusB(BusB)
    );

    // Clock generator
    always #5 clock = ~clock;

    initial begin
        $display("Time\tRW\tBusW\t\tRA\tRB\tBusA\t\t\tBusB");

        // Initialize inputs
        clock = 0;
        RegWr = 0;
        RA = 0; RB = 0; RW = 0; BusW = 0;

        #10;

        // Write 0xAAAA_AAAA to R1
        RegWr = 1; RW = 4'd1; BusW = 32'hAAAA_AAAA;
        #10;

        // Write 0x5555_5555 to R2
        RW = 4'd2; BusW = 32'h5555_5555;
        #10;

        // Disable write
        RegWr = 0;

        // Read R1 and R2
        RA = 4'd1; RB = 4'd2;
        #10;
        $display("%0t\t%d\t%h\t%d\t%d\t%h\t%h", $time, RW, BusW, RA, RB, BusA, BusB);

        // Try writing to R0 (should remain 0 if protected)
        RegWr = 1; RW = 4'd0; BusW = 32'hFFFF_FFFF;
        #10;

        // Read from R0 and R1
        RegWr = 0; RA = 4'd0; RB = 4'd1;
        #10;
        $display("%0t\t%d\t%h\t%d\t%d\t%h\t%h", $time, RW, BusW, RA, RB, BusA, BusB);

        // Write to R7
        RegWr = 1; RW = 4'd7; BusW = 32'h0F0F_0F0F;
        #10;

        // Read from R7 and R2
        RegWr = 0; RA = 4'd7; RB = 4'd2;
        #10;
        $display("%0t\t%d\t%h\t%d\t%d\t%h\t%h", $time, RW, BusW, RA, RB, BusA, BusB);

        $display("\n--- Register File Test Completed ---");
        $finish;
    end

endmodule





