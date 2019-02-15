module fifo_16_by_8(empty, rst, data_in, data_out, rd_inc, wr_inc);
	
	input rst;
	input [23:0] data_in;
	input rd_inc, wr_inc;
	
	reg[2:0] wr_pntr, rd_pntr;
	
	output empty;
	output wire [15:0] data_out;

	reg [23:0] fifo [7:0];
	
	assign empty = (rd_pntr == wr_pntr);
	assign data_out = fifo[rd_pntr];
	
	integer i;
	
	
	//read on write increment
	always @(negedge wr_inc or negedge rst)begin
		if(~rst)begin
			wr_pntr <=0;
		end
		
		fifo[wr_pntr] <= data_in; 
		wr_pntr <= wr_pntr +1;
	end
	
	//read on read increment bit
	always @(negedge rd_inc or negedge rst)begin
		if(~rst) begin
			rd_pntr <=0;
		end
		else if(empty);
		else begin
			rd_pntr <= rd_pntr +1;
		end
	end

endmodule


module fifo_testbench();

endmodule