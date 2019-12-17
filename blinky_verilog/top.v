module top(input clk_100mhz, output [3:1] led);

reg [25:0] counter = 0;
assign led = counter[25:23];
always @(posedge clk_100mhz) counter <= counter + 1;

endmodule