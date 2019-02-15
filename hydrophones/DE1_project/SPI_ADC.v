module SPI_ADC(CLK, RST, EOC, TX_READY, DATAIN, MISO, CS, SCLK, MOSI, RX, RX_READY, TX_SENT);
	parameter divSCLK = 4; 			// Divides 50 MHz down to 5 MHz for SCLK
	parameter readCMd = 24'h1A0000;		// Initial 8 bytes to start SPI read of ADC
												// Contains start bit, read address, and read bit
	input CLK;					// 50 MHz
	input RST; 					// Active-Low Reset
	input EOC; 					// Initiates a 64-bit read from the A, B, C, and D channels of ADC when driven low
	input TX_READY; 			// Will initiate a send of the data present at DATAIN when this is high and FSM is IDLE
	input [23:0] DATAIN; 	// Data to send over SPI to ADC; TX_READY is set high to indicate this should be sent
	input MISO;					// SPI master-in-slave-out
	
	output CS; 					// SPI slave select
	output SCLK; 				// SPI clock (divided by "divSCLK" param, set to 5 MHz right now
	output MOSI;				// SPI master-out-slave-in
	output [63:0] RX; 		// ADC data read from all channels. [63:48] = A, [47:32] = B, [31:16] = C, [15:0] = D
	output RX_READY; 			// Pulses high when RX has valid data read from ADC over SPI
	output TX_SENT;			// Pulses high to indicate DATAIN has been sent over SPI and TX_READY can be dropped low
	
	// SPI Clock line
	reg SCLK; 
	
	// Chip select line
	reg CS; 
	
	// Indication that RX data is valid to read 
	reg RX_READY; 
	
	// Indication that DATAIN has been sent, and TX_READY should be dropped (or DATAIN changed) 
	reg TX_SENT; 

	// FSM States and registers to hold them - one-hot encoding
	parameter IDLE = 4'b0001; 
	parameter BEGIN_SPI_READ = 4'b0010; 
	parameter RECV_DATA = 4'b0100;
	parameter SPI_CMD_SEND = 4'b1000;
	
	reg [3:0] fsm_state; 
	reg [3:0] next_state;
	
	// RX is a big shift register while receiving on SPI
	reg [63:0] RX; 
	reg [7:0] rx_bits_remaining; // Keeps track of how many more bits to receive
	
	// Shift register and count for current byte of data being sent over SPI 
	reg [23:0] tx_out;
	reg [7:0] tx_bits_remaining; 
	assign MOSI = tx_out[23]; 	// TX signal gets MSB of shift register
	
	// Determinations of next state in state machine
	always @(EOC or fsm_state or tx_bits_remaining or rx_bits_remaining or TX_READY) 
	begin
		case (fsm_state)
			IDLE: begin
				if (TX_READY)
					next_state = SPI_CMD_SEND; 
				else if (!EOC) 
					next_state = BEGIN_SPI_READ;
				else
					next_state = IDLE; 
			end
			
			BEGIN_SPI_READ: begin
				if (tx_bits_remaining == 0)
					next_state = RECV_DATA; 
				else
					next_state = BEGIN_SPI_READ; 
			end
			
			RECV_DATA: begin
				if (rx_bits_remaining == 0)
					next_state = IDLE; 
				else 
					next_state = RECV_DATA; 
			end
			
			SPI_CMD_SEND: begin
				if (tx_bits_remaining == 0)
					next_state = IDLE; 
				else
					next_state = SPI_CMD_SEND; 
			end
			
			default:
				next_state = IDLE; 
				
		endcase
	end
	
	// Keeps track of 50 Mhz clock ticks to divide it down to lower frequency
	reg [7:0] clkCount; 
	
	integer i, j; 
	
	// SCLK synchronous ticks (or asynchronously triggered by EOC)
	always @(posedge CLK or negedge RST)
	begin
		if (!RST) begin
			fsm_state <= IDLE; 
			clkCount <= 0; 
			SCLK <= 0; 
			tx_out <= 0; 
			tx_bits_remaining <= 0; 
			RX <= 0;
			RX_READY <= 0; 
			rx_bits_remaining <= 0; 
			TX_SENT <= 1; 
			CS <= 1; 
			end
		else begin
			if (clkCount < (divSCLK / 2)) 	// Increment value in clock divider
				clkCount <= clkCount + 1; 
			else begin								// Clock divider completed clock half cycle
				clkCount <= 0; 
				
				if (!SCLK) begin					// Negative portion of SCLK cycle (just before rising edge)
					
					fsm_state <= next_state; 
					
					if ((fsm_state == IDLE) && (next_state == BEGIN_SPI_READ)) begin			// Just ADC read SPI transaction
						tx_out <= readCMd;
						tx_bits_remaining <= 8; 
						RX_READY <= 0; 
						CS <= 0; 
						end
					else if ((fsm_state == RECV_DATA) && (next_state == IDLE)) begin			// Ending the ADC read SPI transaction
						CS <= 1; 
						RX_READY <= 1; 
						end
					else if ((fsm_state == IDLE) && (next_state == SPI_CMD_SEND)) begin		// Starting CMD send transaction
						tx_out <= DATAIN; 
						tx_bits_remaining <= 16;
						TX_SENT <= 0; 
						CS <= 0; 
						end
					else if ((fsm_state == SPI_CMD_SEND) && (next_state == IDLE)) begin		// Ending the CMD send transaction
						CS <= 1; 
						TX_SENT <= 1; 
						end
					else if ((fsm_state == RECV_DATA) || (next_state == RECV_DATA)) begin	// Either just starting ABCD channel receive, or in middle of it
						
						// Shift everything in receive register by 1, MISO into LSB
						RX[0] <= MISO; 
						for (i = 1; i < 64; i = i + 1)
							RX[i] <= RX[i - 1]; 
						
						if (fsm_state == BEGIN_SPI_READ) 	// Just beginning the receive, so set the remaining RX count
							rx_bits_remaining <= 63; 
						else											// Else decrement it like normal 
							rx_bits_remaining <= rx_bits_remaining - 1; 
							
						SCLK <= ~SCLK;
						end
					else if ((fsm_state == BEGIN_SPI_READ) || (fsm_state == SPI_CMD_SEND)) begin
						SCLK <= ~SCLK;
					end
					
					end
				else begin									// Positive portion of SCLK cycle (just before falling edge)
					SCLK <= ~SCLK; 
					
					// Data change can happen at this part of the cycle
					if ((fsm_state == BEGIN_SPI_READ) || (fsm_state == SPI_CMD_SEND)) begin	
						// Shift the TX register
						tx_out[0] <= 0;
						for (j = 1; j < 24; j = j + 1)
							tx_out[j] <= tx_out[j - 1]; 
						
						// Decrement the number of bits left to send;
						tx_bits_remaining <= tx_bits_remaining - 1;
					end
				end
			end
		end
	end
endmodule