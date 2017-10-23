-----------------------------------------------------------------------------
library ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--------------------------------------------------
entity Encoder is
port(   reset,clk 	:in std_logic;
		start		:in std_logic;
		data		:in std_logic_vector(15 downto 0);
		pulse		:out std_logic
		
    );
end Encoder;
---------------------------------------
architecture func of Encoder is

constant k : INTEGER := 1000;  --2e13
signal counter1: integer range 0 to k;
signal data_temp: std_logic_vector(15 downto 0);
-------------------------------------------
begin
-------------------------------------------

process(reset,clk)
begin
if reset='0' then
	pulse<='1';
	counter1<=1;
	data_temp<=data;
elsif clk'event and clk='1' then
	if start='0' then
		counter1<=counter1+1;
		if counter1<=30 then
			pulse<='0';
			data_temp<=data;
		elsif counter1<=50 and counter1>30 then
			pulse<='1';
		elsif counter1<=70 and counter1>50 then
			pulse<=data_temp(15);
		elsif counter1<=90 and counter1>70 then
			pulse<=data_temp(14);
		elsif counter1<=110 and counter1>90 then
			pulse<=data_temp(13);
		elsif counter1<=130 and counter1>110 then
			pulse<=data_temp(12);
		elsif counter1<=150 and counter1>130 then
			pulse<=data_temp(11);
		elsif counter1<=170 and counter1>150 then
			pulse<=data_temp(10);
		elsif counter1<=190 and counter1>170 then
			pulse<=data_temp(9);
		elsif counter1<=210 and counter1>190 then
			pulse<=data_temp(8);
		elsif counter1<=230 and counter1>210 then
			pulse<=data_temp(7);
		elsif counter1<=250 and counter1>230 then
			pulse<=data_temp(6);
		elsif counter1<=270 and counter1>250 then
			pulse<=data_temp(5);
		elsif counter1<=290 and counter1>270 then
			pulse<=data_temp(4);
		elsif counter1<=310 and counter1>290 then
			pulse<=data_temp(3);
		elsif counter1<=330 and counter1>310 then
			pulse<=data_temp(2);
		elsif counter1<=350 and counter1>330 then
			pulse<=data_temp(1);
		elsif counter1<=370 and counter1>350 then
			pulse<=data_temp(0);
		elsif counter1>370 then
			counter1<=380;
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
