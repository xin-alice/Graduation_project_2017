-----------------------------------------------------------------------------
library ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--------------------------------------------------
entity Decoder is
port(   reset,clk 	:in std_logic;
		pulse		:in std_logic;
		start_test	:out std_logic;
		err_com		:out std_logic;
		data		:buffer std_logic_vector(5 downto 1)
    );
end Decoder;
---------------------------------------
architecture func of Decoder is
CONSTANT k : INTEGER := 50;  --2e13
signal counter1: integer range 0 to k;
signal counter2: integer range 0 to k;
signal counter_down: integer range 0 to 4000;
signal counter_up: integer range 0 to 4000;
signal data_temp:std_logic_vector(5 downto 1);
signal i: integer range 0 to k;
signal pulse_last:std_logic_vector(20 downto 1);
signal parity:std_logic;
signal start, start_last:std_logic;
signal start_group:std_logic_vector(2 downto 1);
signal counter2_start:std_logic;
signal serial_clk:std_logic;
-------------------------------------------
begin
-------------------------------------------
process(reset,clk)
begin
if reset='0' then
	start<='0';
	pulse_last<="00000000000000000000";
	i<=1;
	counter1<=1;
	counter2<=1;
	data_temp<="00000";
	data<="00000";
elsif clk'event and clk='1' then

		pulse_last<=TO_STDLOGICVECTOR(TO_BITVECTOR(pulse_last) SLL 1);
		pulse_last(1)<=pulse;
		if pulse_last="11111111110000000000" then
			counter2_start<='1';
		elsif pulse_last="00000000001111111111" then
			counter2_start<='0';
		end if;
	
		if counter2_start='1' then
			counter2<=counter2+1;
			if counter2>=40 then 
				counter2<=40;
			end if;
		else
			if counter2>=26 and counter2<=34 then
				start<='1';
				counter1<=1;
				i<=1;
			end if;
			counter2<=1;
		end if;

	if  start='1' then
		if counter1=20 then
			data_temp(6-i)<=pulse;
			counter1<=1;
			if i=5 then
				i<=1;
				start<='0';
				data<=data_temp;
			else i<=i+1;
			end if;
		else 
			counter1<=counter1+1;
		end if;
	end if;
end if;
end process;
----------------------------------------------
--判断是否发生通信故障
start_group<=start_last & start;
process(reset, clk)
begin
if reset='0' then
	err_com<='1';
	counter_down<=0;
	counter_up<=0;
	start_last<='0';
elsif clk'event and clk='1' then
	if start_group="00" then
		counter_down<=counter_down+1;
		if counter_down>=2000 then
			counter_down<=2200;
		end if;
	elsif start_group="11" then
		counter_up<=counter_up+1;
		if counter_up>=2000 then 
			counter_up<=2200;
		end if;
	elsif start_group="01" then
		counter_down<=0;
	elsif start_group="10" then
		counter_up<=0;
	end if;
	
	if 	(counter_up<2000 and counter_down<2000) then
		err_com<='0';	
	else
		err_com<='1';
	end if;
	start_last<=start;
end if;
end process;
---------------------------------------------
start_test<=start;
end func;
