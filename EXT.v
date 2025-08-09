	 
module extender (
    input  [13:0] imm_in,     // 14-bit immediate input
    input         ext_ctrl,   // 0 = zero extend, 1 = sign extend
    output [31:0] imm_out     // 32-bit extended output
);
    
    assign imm_out = ext_ctrl ? {{18{imm_in[13]}}, imm_in} : {18'b0, imm_in};
    
endmodule





module sign_extender (
    input  [13:0] imm_in,     // 14-bit immediate input
    output [31:0] imm_out     // 32-bit sign-extended output
);
    
    assign imm_out = {{18{imm_in[13]}}, imm_in};
    
endmodule

// Zero Extender Module  
// Extends 14-bit immediate to 32-bit with zero extension
module zero_extender (
    input  [13:0] imm_in,     // 14-bit immediate input
    output [31:0] imm_out     // 32-bit zero-extended output
);
    
    assign imm_out = {18'b0, imm_in};
    
endmodule



// Testbench for Sign Extender
module tb_sign_extender;
    
    reg  [13:0] imm_in;
    wire [31:0] imm_out;
    
    // Instantiate the sign extender
    sign_extender uut (
        .imm_in(imm_in),
        .imm_out(imm_out)
    );
    
    initial begin
        $display("Testing Sign Extender");
        $display("Time\tInput(14-bit)\t\tOutput(32-bit)");
        $display("----\t-------------\t\t--------------");
        
        // Test positive numbers (MSB = 0)
        imm_in = 14'b00000000000001; // +1
        #10;
        $display("%0t\t%b\t%b", $time, imm_in, imm_out);
        
        imm_in = 14'b01111111111111; // +8191 (largest positive)
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b00000000001010; // +10
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        // Test negative numbers (MSB = 1)
        imm_in = 14'b11111111111111; // -1
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b10000000000000; // -8192 (most negative)
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b11111111110110; // -10
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        // Test zero
        imm_in = 14'b00000000000000; // 0
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        $display("\nSign Extender Test Complete\n");
        $finish;
    end
    
endmodule

// Testbench for Zero Extender
module tb_zero_extender;
    
    reg  [13:0] imm_in;
    wire [31:0] imm_out;
    
    // Instantiate the zero extender
    zero_extender uut (
        .imm_in(imm_in),
        .imm_out(imm_out)
    );
    
    initial begin
        $display("Testing Zero Extender");
        $display("Time\tInput(14-bit)\t\tOutput(32-bit)");
        $display("----\t-------------\t\t--------------");
        
        // Test various values
        imm_in = 14'b00000000000001; // 1
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b01111111111111; // 8191
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b10000000000000; // 8192 (MSB=1, but zero extended)
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b11111111111111; // All 1s
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b00000000000000; // 0
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        imm_in = 14'b10101010101010; // Alternating pattern
        #10;
        $display("%0t\t%b\t%h", $time, imm_in, imm_out);
        
        $display("\nZero Extender Test Complete\n");
        $finish;
    end
    
endmodule

// Testbench for Combined Extender
module tb_extender;
    
    reg  [13:0] imm_in;
    reg         ext_ctrl;
    wire [31:0] imm_out;
    
    // Instantiate the combined extender
    extender uut (
        .imm_in(imm_in),
        .ext_ctrl(ext_ctrl),
        .imm_out(imm_out)
    );
    
    initial begin
        $display("Testing Combined Extender");
        $display("Time\tInput(14-bit)\t\tCtrl\tOutput(32-bit)\t\tType");
        $display("----\t-------------\t\t----\t--------------\t\t----");
        
        // Test with positive number
        imm_in = 14'b00000000001010; // +10
        ext_ctrl = 0; // Zero extend
        #10;
        $display("%0t\t%b\t%b\t%h\t\tZero", $time, imm_in, ext_ctrl, imm_out);
        
        ext_ctrl = 1; // Sign extend
        #10;
        $display("%0t\t%b\t%b\t%h\t\tSign", $time, imm_in, ext_ctrl, imm_out);
        
        // Test with negative number
        imm_in = 14'b11111111110110; // -10 in 2's complement
        ext_ctrl = 0; // Zero extend
        #10;
        $display("%0t\t%b\t%b\t%h\t\tZero", $time, imm_in, ext_ctrl, imm_out);
        
        ext_ctrl = 1; // Sign extend
        #10;
        $display("%0t\t%b\t%b\t%h\t\tSign", $time, imm_in, ext_ctrl, imm_out);
        
        // Test with all 1s
        imm_in = 14'b11111111111111; // All 1s
        ext_ctrl = 0; // Zero extend
        #10;
        $display("%0t\t%b\t%b\t%h\t\tZero", $time, imm_in, ext_ctrl, imm_out);
        
        ext_ctrl = 1; // Sign extend
        #10;
        $display("%0t\t%b\t%b\t%h\t\tSign", $time, imm_in, ext_ctrl, imm_out);
        
        $display("\nCombined Extender Test Complete\n");
        $finish;
    end
    
endmodule

