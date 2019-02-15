library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity UART is
port(
CLOCK_50: 	in std_logic;
BYTE: 		out std_logic_vector(7 downto 0);
LEDR:			out std_logic_vector(7 downto 0); 
UART_TXD: 	out std_logic;
UART_RXD: 	in std_logic;
RX_COMPLETE:		out std_logic
);
end UART;

architecture BEHAV of UART is

signal TX_DATA: 	std_logic_vector(7 downto 0);
signal TX_START: 	std_logic:='0';
signal TX_BUSY: 	std_logic;
signal RX_DATA: 	std_logic_vector(7 downto 0);
signal RX_BUSY: 	std_logic;

----------------------------------------------------------------
component TX
port(
CLK: 				in std_logic;
START: 			in	std_logic;
BUSY: 			out std_logic;
DATA: 			in	std_logic_vector(7 downto 0);
TX_LINE: 		out std_logic
);
end component TX;

-----------------------------------------------------------------
component Rx
port(
CLK: in std_logic;
RX_LINE: in std_logic;
DATA: out std_logic_vector(7 downto 0);
BUSY: out std_logic
);
end component RX;

------------------------------------------------------------------

begin


--C1: TX port map (CLOCK_50, TX_START, TX_BUSY, TX_DATA, UART_TXD);
C2: RX port map (CLOCK_50, UART_RXD, RX_DATA, RX_BUSY);

process (RX_BUSY)
begin
RX_COMPLETE <= RX_BUSY;
if (RX_BUSY' event and RX_BUSY = '0') then
	BYTE(7 downto 0) <= RX_DATA(7 downto 0);
	LEDR(7 downto 0) <= RX_DATA(7 downto 0);
end if;

end process;

end BEHAV;