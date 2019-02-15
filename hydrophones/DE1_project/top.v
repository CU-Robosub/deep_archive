module top(	input CLOCK_50,	//50 MHZ clock
				output SHDN,		//shtdown pin active high
				input EOC,			//End of COnversion
				output SCLK,		//serial clock for SPI
				output DIN_11043, //Data in for 11043
				input DOUT_11043,	//Data out of 11043
				output CS,			//chip select for 11043
				output reg CONV_RUN,
				output DAC_STEP,
				output UP_DWN,
				input UART_RX,
				input [3:0]KEY,
				input [9:0]SW,
				output [9:0]LEDR,
				output [6:0]HEX0,
				output [6:0]HEX1,
				output [6:0]HEX2,
				output [6:0]HEX3,
				output [6:0]HEX4,
				output [6:0]HEX5);


wire [63:0] SPI_OUT;
wire SYS_RESET = KEY[0];
assign SHDN = 0;
assign DAC_STEP = 0;

reg [23:0] command_data;

reg tx_ready;
wire tx_sent;

initial begin
	CONV_RUN = 0;
	tx_ready = 0;
	uart_rx_cnt =0;
end

wire uart_rx_trigger;
reg [1:0] uart_rx_cnt;
reg[23:0] uart_24_reg;
wire[7:0] uart_data;
reg fifo_wrt_trg;

SPI_ADC SPI_11043(.CLK(CLOCK_50),
						.RST(SYS_RESET),
						.EOC(EOC),
						.TX_READY(tx_ready),
						.DATAIN(command_data),
						.MISO(DOUT_11043),
						.CS(CS),
						.SCLK(SCLK),
						.MOSI(DIN_11043),
						.RX(SPI_OUT),
						.RX_READY(),  
						.TX_SENT(tx_sent));
		

		
UART UART_PC(	.CLOCK_50(CLOCK_50),
					.BYTE(uart_data),
					.LEDR(),
					.UART_TXD(),
					.UART_RXD(UART_RX),
					.RX_COMPLETE(uart_rx_trigger));
					
					
fifo_16_by_8 FIFO(.empty(),
						.rst(SYS_RESET),
						.data_in(uart_24_reg[23:0]),
						.data_out(),
						.rd_inc(),
						.wr_inc(fifo_wrt_trg));

hexto7segment  BYTE0(.x(uart_24_reg[7:0]),
							.z1(HEX0[6:0]),
							.z2(HEX1[6:0]));

hexto7segment  BYTE1(.x(uart_24_reg[15:8]),
							.z1(HEX2[6:0]),
							.z2(HEX3[6:0]));
				

hexto7segment  BYTE2(.x(uart_24_reg[23:16]),
							.z1(HEX4[6:0]),
							.z2(HEX5[6:0]));

							
assign LEDR[1:0] = uart_rx_cnt;
assign LEDR[9] = uart_rx_trigger; 
							
//3 bytes block
always @(posedge uart_rx_trigger)begin
	if(uart_rx_cnt == 2'b00)begin
		uart_24_reg[7:0] <= uart_data;
		uart_rx_cnt <= uart_rx_cnt +1;
		fifo_wrt_trg <= 0;
	end
	
	else if(uart_rx_cnt == 2'b01)begin
		uart_24_reg[15:8] <= uart_data;
		uart_rx_cnt <= uart_rx_cnt +1;
	end
		
	else if(uart_rx_cnt == 2'b10)begin
		uart_24_reg[23:16] <= uart_data;
		uart_rx_cnt <= 0;
		//latch into fifo here
		fifo_wrt_trg <= 1;
	end
	else uart_rx_cnt <= 0;
	
end					
							
							
							
							
/*							
							
//////////////////////
///garbage code
////////////////////////
reg [3:0] cmd_tst_fsm;
reg [3:0] adc_setup_pntr;
reg [23:0] initial_adc_reg [3:0];
reg flag;
initial begin
	cmd_tst_fsm = 4'b0001;
	adc_setup_pntr = 0;
	initial_adc_reg[0] = 24'h30101B;
	initial_adc_reg[1] = 24'h34101B;
	initial_adc_reg[2] = 24'h38101B;
	initial_adc_reg[3] = 24'h3C101B;
end

parameter HOLD_START =	4'b0001;
parameter INIT_ADC 	= 	4'b0010;
parameter READ_DATA 	= 	4'b0100; 

	
//assign LEDR[3:0] = cmd_tst_fsm;
//assign LEDR[9:5] = adc_setup_pntr;

always @(posedge CLOCK_50)begin
	
	if(~SYS_RESET)begin
		cmd_tst_fsm <= HOLD_START;
		tx_ready <=0;
		CONV_RUN <= 0;
		adc_setup_pntr <= 0;
	end
	

	case(cmd_tst_fsm)
		HOLD_START: begin
			if(~KEY[3])
				cmd_tst_fsm <= INIT_ADC;
		end
		
		INIT_ADC: begin
			if(adc_setup_pntr == 4)
				cmd_tst_fsm <= READ_DATA;
			
			else
				if(tx_sent)begin
					flag = 1;
					command_data <= initial_adc_reg[adc_setup_pntr];
					tx_ready <= 1;
				end
				
				else begin
					if(flag)begin
						adc_setup_pntr <= adc_setup_pntr +1;
						flag = 0;
					end	
				end
			
		end
		
		READ_DATA: begin
			CONV_RUN <= 1;
			tx_ready <=0;
		end
		
	
	endcase
end
	*/			
				
				

endmodule




















