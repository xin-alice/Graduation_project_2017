library ieee;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--================================--
entity phase_ctrl is
port(   clk		:in std_logic;
		reset	:in std_logic;
		
		data_out:buffer std_logic;
		pwm_in	:in std_logic;
		
		pwma	:buffer std_logic_vector(8 downto 1);
		pwmn	:buffer std_logic_vector(8 downto 1);
		flt		:in std_logic_vector(8 downto 1);
		
		AD_clk		:buffer std_logic;
		AD_OTR		:in std_logic;
		AD_TriState	:out std_logic;
		AD_STBY		:out std_logic;
		AD_data		:in std_logic_vector(9 downto 0);
		
		SW		:in std_logic_vector(8 downto 1);
		io		:inout std_logic_vector(10 downto 1);
		state	:buffer std_logic;
		LED		:out std_logic_vector(8 downto 1);
		rest	:out std_logic_vector(7 downto 1)
	);
end phase_ctrl;
-----------------------------------------------------------------------------------------------
architecture func of phase_ctrl is

	CONSTANT value_LED	:INTEGER:=20000000;
	CONSTANT value_AD	:INTEGER:=2000;
	signal	counter_LED	:integer range 0 to value_LED;
	signal	counter_AD	:integer range 0 to value_AD;
	signal	LED_temp	: std_logic_vector(8 downto 1);
	signal	LED_temp_err: std_logic_vector(8 downto 1);
	signal	LED_flag	: std_logic;
	signal	pwm			: std_logic_vector(3 downto 1);
	signal	pwmad		: std_logic_vector(3 downto 1);
	signal	pwmnd		: std_logic_vector(3 downto 1);
	signal	data_temp	: std_logic_vector(15 downto 0):=X"0000";
	
	signal	start_test	: std_logic;
	signal	enableP, enableN	: std_logic;
	signal	pwm_EnP, pwm_EnN	: std_logic;
	signal	err_drv		: std_logic;
	signal	err_com		: std_logic;
	signal	data_receive: std_logic_vector(5 downto 1);
	
component delay
port(   pwm_in :in std_logic;
     	reset,clk :in std_logic;
       pwm_out :out std_logic
    );
end component;

component Decoder
port(   reset,clk 	:in std_logic;
		pulse		:in std_logic;
		start_test	:out std_logic;
		err_com		:out std_logic;
		data		:buffer std_logic_vector(5	downto 1)
    );
end component;

component Encoder is
port(   reset,clk 	:in std_logic;
		start	:in std_logic;
		data	:in std_logic_vector(15	downto 0);
		pulse	:out std_logic
    );
end component;
-----------------------------------------------------------------------------------------------------
begin
----------------------------------------------------------------------------------------------------
--跑马灯
horse_race_lamp:process(clk,reset)
begin
if reset='0' then
	counter_LED<=0;
	LED_temp<="00000001";
	LED_flag<='0';
elsif clk'event and clk='1' then
	if counter_LED = value_LED then 
		counter_LED<=0 ;
		LED_temp<=TO_STDLOGICVECTOR(TO_BITVECTOR(LED_temp) ROL 1);
		LED_flag<= not LED_flag;
	else counter_LED<=(counter_LED+1);
	end if;
end if;
end process;

err_warning:process(clk,reset)
begin
if reset='0' then
	LED_temp_err<="00000000";
elsif clk'event and clk='1' then
	if LED_flag = '0' then
		if err_drv='1' then 
			LED_temp_err<=(not flt);
		elsif err_com='1' then 
			LED_temp_err<="11110000";
		elsif (pwm_EnP and pwm_EnN) ='0' then
			LED_temp_err<="10101010";
		else
			LED_temp_err<="00000000";
		end if;
	else 
		if err_drv='1' then 
			LED_temp_err<="00000000";
		elsif err_com='1' then 
			LED_temp_err<="00001111";
		elsif (pwm_EnP and pwm_EnN)='0' then
			LED_temp_err<="01010101";
		else
			LED_temp_err<="00000000";
		end if;
	end if;
end if;
end process;
---------------------------------------------
Led_lighting:process(clk,reset)
begin
if reset='0' then
	LED<="11111111";
elsif clk'event and clk='1' then
	if SW = "10000000" then
		LED<=AD_data(7 downto 0);
	elsif SW = "11000000" then
		LED<=flt;
	elsif (enableP and enableN) ='0' then
		LED<=LED_temp_err;
	else
		LED(4 downto 1)<=LED_temp(4 downto 1);
		LED(5)<=LED_temp(8);
		LED(6)<=LED_temp(7);
		LED(7)<=LED_temp(6);
		LED(8)<=LED_temp(5);
		--LED<=AD_data(7 downto 0);
	end if;
end if;
end process;
--------------------------------------------
AD_STBY<='0';
AD_TriState<='0';
-----------------------------------------------------------------------------------------------------
----产生AD时钟
AD_clk_generation:process(clk,reset)
begin
if reset='0' then
	counter_AD<=0;
	AD_clk<='0';
elsif clk'event and clk='1' then
	if counter_AD = value_AD then 
		counter_AD<=0 ;
		AD_clk<=not AD_clk;
	else counter_AD<=(counter_AD+1);
	end if;
end if;
end process;
------------------------------------------------------------------------------------------------------
--死区生成，将所有脉冲延时一段时间导通，关断不延时
PWM_delay1:delay port map
(
 pwm_in=>pwm(1),
 reset=>reset,
 clk=>clk,
 pwm_out=>pwmad(1)
);

PWM_delay2:delay port map
(
 pwm_in=>not pwm(1),
 reset=>reset,
 clk=>clk,
 pwm_out=>pwmnd(1)
);

PWM_delay3:delay port map
(
 pwm_in=>pwm(2),
 reset=>reset,
 clk=>clk,
 pwm_out=>pwmad(2)
);

PWM_delay4:delay port map
(
 pwm_in=>not pwm(2),
 reset=>reset,
 clk=>clk,
 pwm_out=>pwmnd(2)
);

PWM_delay5:delay port map
(
 pwm_in=>pwm(3),
 reset=>reset,
 clk=>clk,
 pwm_out=>pwmad(3)
);

PWM_delay6:delay port map
(
 pwm_in=>not pwm(3),
 reset=>reset,
 clk=>clk,
 pwm_out=>pwmnd(3)
);
------------------------------------------------------------------------------------------------------
--产生正负驱动脉冲信号，enable为1则使能PWM，否则封锁
pwma(1)<=pwmad(1) and enableP;
pwma(2)<=pwmnd(1) and enableN and enableP;
pwma(3)<=pwmad(1) and enableP and enableN;
pwma(4)<=pwmnd(1) and enableN;
pwma(5)<=pwmad(2) and enableP;
pwma(6)<=pwmad(3) and enableP;
pwma(7)<=pwmnd(3) and enableN;
pwma(8)<=pwmnd(2) and enableN;

pwmn<=not pwma;
---------------------------------------------------------------------------------------------------------
--接收PWM信号并解码，首位是使能信号，1使能0封锁，flt为驱动保护信号，1正常0保护，err_drv是驱动故障信号，0正常
--err_com是通信故障信号，0正常，state是相状态信号，1正常，灯亮
receive:Decoder port map
(
 pulse=>(not pwm_in),
 reset=>reset,
 clk=>clk,
 start_test=>start_test,
 err_com=>err_com,
 data=>data_receive
);

pwm_EnP<=data_receive(5);
pwm_EnN<=data_receive(4);
pwm<=data_receive(3 downto 1);
err_drv<=not (flt(1) and flt(2) and flt(3) and flt(4) and flt(5) and flt(6) and flt(7) and flt(8));
state<=not (err_com or err_drv);
enableP<=pwm_EnP and state;
enableN<=pwm_EnN and state;
--------------------------------------------------------------------------------------------------------------
--串行发送故障信号和AD采样数据
data_temp(15)<=state;
data_temp(14 downto 10)<="00000";
data_temp(9 downto 0)<=AD_data;
--data_temp<=X"5555";

send:Encoder port map
(
 pulse=>data_out,
 reset=>reset,
 clk=>clk,
 start=>AD_clk,
 data=>data_temp
);
--------------------------------------------------------------------------------------------------------------
rest<="0000000";
IO(1)<=pwm_EnN;
IO(2)<=data_receive(3);
IO(3)<=data_receive(2);
IO(4)<=data_receive(1);
--------------------------------------------------
end func;

