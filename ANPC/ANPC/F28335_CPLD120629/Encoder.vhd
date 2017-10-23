-----------------------------------------------------------------------------
library ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--------------------------------------------------
entity Encoder is
port(   reset,clk 	:in std_logic;
		start		:in std_logic;
		data		:in std_logic_vector(5 downto 1);
		pulse		:out std_logic
		
    );
end Encoder;
---------------------------------------
architecture func of Encoder is

signal counter1: integer range 0 to 200;
signal data_temp:std_logic_vector(5 downto 1);
-------------------------------------------
begin
-------------------------------------------
process(reset,clk)
begin
if reset='0' or start='0' then
	pulse<='1';
	counter1<=1;
	data_temp<=data;
elsif clk'event and clk='1' and start='1' then
	if start='1' then
		counter1<=counter1+1;
		if counter1<=30 then
			pulse<='0';
			data_temp<=data;
		elsif counter1<=50 and counter1>30 then
			pulse<='1';
		elsif counter1<=70 and counter1>50 then
			pulse<=data_temp(5);
		elsif counter1<=90 and counter1>70 then
			pulse<=data_temp(4);
		elsif counter1<=110 and counter1>90 then
			pulse<=data_temp(3);
		elsif counter1<=130 and counter1>110 then
			pulse<=data_temp(2);
		elsif counter1<=150 and counter1>130 then
			pulse<=data_temp(1);	
		elsif counter1>150 then
			counter1<=160;
			pulse<='1';
		else
			pulse<='1';
		end if;
	else
		pulse<='1';
		counter1<=1;
	end if;
end if;
end process;
---------------------------------------------
end func;
