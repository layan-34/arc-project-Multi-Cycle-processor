 `timescale 1ns/1ps			      
module Alu(
    input [31:0] A, B,
    input [3:0] ALU_Control,
	input EN,
    output reg [31:0] Y,
    output reg Zero, Negative, Positive
);

	reg signed [15:0] signed_A, signed_B, signed_Y;

    always @(*) begin  
		if (EN) begin
		
        case (ALU_Control)
            4'b0000: Y = A | B; // or
            4'b0001: Y = A + B; // ADD
            4'b0010: Y = A - B; // SUB
            4'b0011:if (A == B)
                        Y = 32'b0;                // Equal
                    else if ($signed(A) < $signed(B))
                        Y = 32'hFFFFFFFF;         // A < B (all 1s = -1)
                    else
                        Y = 32'h00000001;         // A > B (+1)
                	
			4'b0100: Y = A | B;			
			4'b0101 :Y = A + B;
			4'b0110: Y = A + B;
			4'b0111: Y = A + B;
			4'b1000:Y = A + B;
			4'b1001:Y = A + B; 
			4'b1010:  Y = A;
			4'b1011:	 Y = A;
			4'b1100:	 Y = A;	  
		
			
            default: Y = 31'b0; // default to 0, should not happen
        endcase	  
	end
    end
    assign Zero = (Y == 32'b0);                  // Zero flag
    assign Negative = Y[31];                     // Negative flag (MSB)
    assign Positive = (~Y[31]) & (~Zero);
	
    
endmodule 



				   

module Alu_tb;

  // Testbench signals
  reg [31:0] A, B;
  reg [3:0] ALU_Control;
  reg EN;
  wire [31:0] Y;
  wire Zero, Negative, Positive;

  // Instantiate the ALU
  Alu uut (
    .A(A),
    .B(B),
    .ALU_Control(ALU_Control),
    .EN(EN),
    .Y(Y),
    .Zero(Zero),
    .Negative(Negative),
    .Positive(Positive)
  );

  initial begin
    $dumpfile("Alu.vcd");   // For waveform
    $dumpvars(0, Alu_tb);

    EN = 1;

    // Test 1: OR
    A = 32'h0F0F0F0F; B = 32'h00FF00FF; ALU_Control = 6'b000000; #10;
    $display("OR: A=%h B=%h -> Y=%h Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 2: ADD
    A = 32'd10; B = 32'd20; ALU_Control = 6'b000001; #10;
    $display("ADD: A=%d B=%d -> Y=%d Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 3: SUB
    A = 32'd50; B = 32'd30; ALU_Control = 6'b000010; #10;
    $display("SUB: A=%d B=%d -> Y=%d Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 4: CMP (Equal)
    A = 32'd25; B = 32'd25; ALU_Control = 6'b000011; #10;
    $display("CMP (==): A=%d B=%d -> Y=%h Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 5: CMP (A < B)
    A = -32'd5; B = 32'd10; ALU_Control = 6'b000011; #10;
    $display("CMP (<): A=%d B=%d -> Y=%h Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 6: CMP (A > B)
    A = 32'd20; B = 32'd10; ALU_Control = 6'b000011; #10;
    $display("CMP (>): A=%d B=%d -> Y=%h Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 7: Default case with unsupported control
    ALU_Control = 6'b111111; #10;
    $display("DEFAULT: A=%d B=%d -> Y=%h Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    // Test 8: EN disabled
    EN = 0; A = 32'd123; B = 32'd456; ALU_Control = 6'b000001; #10;
    $display("EN=0: A=%d B=%d -> Y=%h Zero=%b Neg=%b Pos=%b", A, B, Y, Zero, Negative, Positive);

    $finish;
  end

endmodule
 