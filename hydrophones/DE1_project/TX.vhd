library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity TX is
port(
CLK: in std_logic;
START: in std_logic;
BUSY: out std_logic;
DATA: in std_logic_vector(7 downto 0);
TX_LINE: out std_logic
);
end TX;

architecture BEHAV of TX is

signal PRSCL: integer range 0 to 5000 :=0;
signal INDEX: integer range 0 to 9 :=0;
signal DATAFLL: std_logic_vector(9 downto 0);
signal TXFLG: std_logic:='0';

begin

process (ClK)
begin
if(CLK'event and clk ='1') then
	if(TXFLG ='0' and START ='1') then
		TXFLG <= '1';
		BUSY <= '1';
		DATAFLL(0) <= '0';
		DATAFLL(9) <= '1';
		DATAFLL(8 downto 1) <= DATA;
	end if;

	if(TXFLG= '1') then
		if(PRSCL <5207) then
			PRSCL <= PRSCL + 1;
		else
			PRSCL <= 0;
		end if;
		
		if (PRSCL =2607) then
			TX_LINE <= DATAFLL(INDEX);
			if (INDEX <9) then
				INDEX <= INDEX +1;
			else
				TXFLG <= '0';
				BUSY <= '0';
				INDEX <= 0;
			end if;
		end if;		
end if;
end if;
end process;
end BEHAV;