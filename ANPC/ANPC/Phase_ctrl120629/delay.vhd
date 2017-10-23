
------------------------------------------------------------------------------
library ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--------------------------------------------------
entity delay is
port(   pwm_in :in std_logic;
     reset,clk :in std_logic;
       pwm_out :out std_logic
--counter_down_f:buffer std_logic
    );
end delay;
---------------------------------------
architecture func of delay is
-------------------------------------------
constant k : INTEGER := 300; 
constant k1: integer := 280;  
constant k2: integer := 8; 
signal   counter_up:integer range 0 to k;
signal 	 counter_down:integer range 0 to k;
--=====================================--
signal	counter_up_f:std_logic;
signal	counter_down_f:std_logic;
signal	pwm_in2,pwm_in1,pwm_in0:std_logic;
signal	counter_up_reset,counter_down_reset	:std_logic;
--=====================================--
begin
-------------------------------------------
pwm_in_edge:process(reset,pwm_in,clk)
begin
if reset='0' then
   pwm_in2<='0';
   pwm_in1<='0';
   pwm_in0<='0';
elsif clk'event and clk='1' then
   pwm_in2<=pwm_in1;
   pwm_in1<=pwm_in0;
   pwm_in0<=pwm_in;    --3¸öÒÆÎ»
end if;
end process;

up_flag:process(reset,clk,counter_up)
begin
if reset='0' then
   counter_up_f<='0';
elsif clk'event and clk='1' then
   if counter_up=k1+1 then 
      counter_up_f<='0' ;
   elsif pwm_in2='0' and pwm_in1='1' and pwm_in0='1' then
      counter_up_f<='1';
   end if;
end if;
end process;
--------------------------------------------
down_flag:process(reset,clk,counter_down)
begin
if reset='0' then
   counter_down_f<='0' ;
elsif clk'event and clk='1' then
   if counter_down=k1+1 then
      counter_down_f<='0' ;
   elsif pwm_in2='1' and pwm_in1='1' and pwm_in0='0' then
      counter_down_f<='1';
   end if;
end if;
end process;
-----------------------------------------------
counter_up1:process(clk,counter_up_reset)
begin
if reset='0' or counter_up_f='0' then
   counter_up<=0;
elsif clk'event and clk='1' then
      counter_up<=counter_up+1;
end if;
end process;
--============================================--
counter_down1:process(clk,counter_down_reset)
begin
if reset='0' or counter_down_f='0' then
   counter_down<=0;
elsif clk'event and clk='1' then
      counter_down<=counter_down+1;
end if;
end process;
--======================================--
pwm_out1:process(reset,clk,pwm_in,counter_up,counter_down)
begin
if reset='0' then
  --pwm_out<=pwm_in;
	pwm_out<='0';
elsif clk'event and clk='1' then
   if counter_up>=k1 then
      pwm_out<='1';
   elsif counter_down>=k2 then
      pwm_out<='0';
   end if;
end if;
end process;
-----------------------------------------------------------------------
counter_down_reset<=reset and counter_down_f;
counter_up_reset<=reset and counter_up_f;

end func;