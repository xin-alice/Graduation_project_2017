--file name:  Detect.vhd;
--  version:  0.0;
--     date:  Jan 7,2008;
--    model:  ;
--
--  company:  Tsinghua University;
--
-- position:  main-board;
--
--  declare:  This design will used to produce the  signals followed:
--            1// 
--            2// 
------------------------------------------------------------------------------
library ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--------------------------------------------------
entity Detect is
port(   err_in :in std_logic;
     reset,clk :in std_logic;
       err_out :out std_logic
--counter_down_f:buffer std_logic
    );
end Detect;
---------------------------------------
architecture func of Detect is
-------------------------------------------
signal	err_in9,err_in8,err_in7,err_in6,err_in5,err_in4,err_in3,err_in2,err_in1,err_in0:std_logic;
--=====================================--
begin
-------------------------------------------
err_in_edge:process(reset,err_in,clk)
begin
if reset='0' then
   err_in9<='0';
   err_in8<='0';
   err_in7<='0';
   err_in6<='0';
   err_in5<='0';
   err_in4<='0';
   err_in3<='0';
   err_in2<='0';
   err_in1<='0';
   err_in0<='0';
elsif clk'event and clk='1' then
   err_in9<=err_in8;
   err_in8<=err_in7;
   err_in7<=err_in6;
   err_in6<=err_in5;
   err_in5<=err_in4;
   err_in4<=err_in3;
   err_in3<=err_in2;
   err_in2<=err_in1;
   err_in1<=err_in0;
   err_in0<=err_in;    --10¸öÒÆÎ»
end if;
end process;
----------------------------------------------------
err_out1:process(reset,clk)
begin
if reset='0' then
   err_out<='1';
elsif clk'event and clk='1' then
   if err_in9='1' and err_in8='1' and err_in7='1' and err_in6='1' and err_in5='1' and err_in4='1' and err_in3='1' and err_in2='1' and err_in1='1' and err_in0='1' then
      err_out<='1';
   elsif err_in9='0' and err_in8='0' and err_in7='0' and err_in6='0' and err_in5='0' and err_in4='0' and err_in3='0' and err_in2='0' and err_in1='0' and err_in0='0' then
      err_out<='0';
   end if;
end if;
end process;
-----------------------------------------------------------------------

end func;