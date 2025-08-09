module mux4(input [3:0] d0, d1, d2, d3,
	input [1:0] s,
	output reg [3:0] y);

	always @( * )
	case(s)
	2'b00: y <= d0;
	2'b01: y <= d1;
	2'b10: y <= d2;
	2'b11: y <= d3;
	endcase
endmodule	 



module mux432b(input [31:0] d0, d1, d2, d3,
	input [1:0] s,
	output reg [31:0] y);

	always @( * )
	case(s)
	2'b00: y <= d0;
	2'b01: y <= d1;
	2'b10: y <= d2;
	2'b11: y <= d3;
	endcase
endmodule






module mux2(input [15:0] d0, d1,
	input s,
	output [15:0] y);
	assign y = s ? d1 : d0;
endmodule



`timescale 1ns/1ps

module mux4_tb;

  // Testbench signals
  reg [3:0] d0, d1, d2, d3;
  reg [1:0] s;
  wire [3:0] y;

  // Instantiate the module under test
  mux4 uut (
    .d0(d0), .d1(d1), .d2(d2), .d3(d3),
    .s(s),
    .y(y)
  );

  // Stimulus block
  initial begin
  

    // Initialize data lines
    d0 = 4'b0001;
    d1 = 4'b0010;
    d2 = 4'b0100;
    d3 = 4'b1000;

    // Cycle through selector values
    s = 2'b00; #10;
    s = 2'b01; #10;
    s = 2'b10; #10;
    s = 2'b11; #10;

    $finish;
  end

endmodule
