module pc (
    input clk,
    input disablepc,
    input [31:0] address,
    output reg [31:0] next_address
);

    always @(posedge clk) begin
        if (!disablepc)
            next_address <= address;
        else
            next_address <= next_address; // Hold the current value
    end

endmodule





module pc_tb;

    reg clk;
    reg disablepc;
    reg [31:0] address;
    wire [31:0] next_address;

    // Instantiate the Program Counter
    pc uut (
        .clk(clk),
        .disablepc(disablepc),
        .address(address),
        .next_address(next_address)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10 ns clock period
    end

    // Test sequence
    initial begin
        $display("Time\tclk\tdisable\taddress\tnext_address");
        $display("------------------------------------------------");

        disablepc = 0;
        address = 32'h00000000;
        #10;

        address = 32'h00000010; // Load address
        #10;

        disablepc = 1;            // Disable PC update
        address = 32'h00000020;
        #10;

        disablepc = 0;            // Enable update again
        address = 32'h00000030;
        #10;

        address = 32'h00000040;
        #10;

        disablepc = 1;            // Hold again
        address = 32'h00000050;
        #10;

        $finish;
    end

    // Monitor outputs
    always @(posedge clk) begin
        $display("%0t\t%b\t%b\t%h\t%h", $time, clk, disablepc, address, next_address);
    end

endmodule
