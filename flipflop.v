module flipflop32 (  
    output reg [31:0] out,
    input clk,
    input writeEn,
    input [31:0] in,
    input reset
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            out <= 32'b0;
        end else if (writeEn) begin
            out <= in;
        end
    end
endmodule