--***************************************************************************************************
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE	IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
--***************************************************************************************************
ENTITY F28335_CPLD IS
	PORT(
--------------------------------------------------------------------------------
		reset                   : IN	STD_LOGIC;
		DSP_clk                 : IN	STD_LOGIC;
		clk                 	: IN	STD_LOGIC;
		MCLKXA                  : IN	STD_LOGIC;
--------------------------------------------------------------------------------
-- bus of DSP
		ADD						: IN 	STD_LOGIC_VECTOR(8 downto 0);
		XA19					: IN	STD_LOGIC;		--2812 IO PORT
        DATA                    : INOUT	STD_LOGIC_VECTOR(15 downto 0);
		ZCS0n     				: IN	STD_LOGIC;
		ZCS7n     				: IN	STD_LOGIC;
		WEn						: IN	STD_LOGIC;
		RDn						: IN	STD_LOGIC;
		RnW  					: IN	STD_LOGIC;
--------------------------------------------------------------------------------
		PDPINTA					: OUT	STD_LOGIC;			--TO 28335 IO PORT(PDPINTA, GPIO12, Pin21)
		PDPINTB					: OUT	STD_LOGIC;			--TO 28335 IO PORT(PDPINTB, GPIO13, Pin24)
		OC						: OUT	STD_LOGIC;			--TO 28335 IO PORT(ECAP1, GPIO34, Pin142)
		ECAP5					: OUT	STD_LOGIC;			--TO 28335 IO PORT(ECAP5, GPIO48, Pin88)
		ECAP6					: OUT	STD_LOGIC;			--TO 28335 IO PORT(ECAP6, GPIO49, Pin89)
---------------------------------------------------------------------------------
-- usb	???	
		USBFLAGA				: IN	STD_LOGIC;
		USBFLAGB				: IN	STD_LOGIC;
		USBFLAGC				: IN	STD_LOGIC;
		USBINT0		    		: IN	STD_LOGIC;
		PKTEND			    	: IN	STD_LOGIC;
		USBCSn			    	: OUT	STD_LOGIC;
--------------------------------------------------------------------------------
-- protection of inverter
		FOB						: IN	STD_LOGIC;			--From J12 of peripheral
		FOA						: IN	STD_LOGIC;			--From J13 of peripheral
		OCA						: IN	STD_LOGIC;			--From Analog
		OCB						: IN	STD_LOGIC;			--From Analog
		OVA						: IN	STD_LOGIC;			--From P1 of CPLD
		OVB						: IN	STD_LOGIC;			--From P1 of CPLD	
		PWMAEN					: buffer	STD_LOGIC;		--TO 244 of peripheral
		PWMBEN					: buffer	STD_LOGIC;		--TO 244 of peripheral
--------------------------------------------------------------------------------
--PWM from DSP
		pwm1A					: IN	STD_LOGIC;
        pwm1B					: IN	STD_LOGIC;
		pwm2A					: IN	STD_LOGIC;
		pwm2B					: IN	STD_LOGIC;
		pwm3A					: IN	STD_LOGIC;
		pwm3B					: IN	STD_LOGIC;
		pwm4A					: IN	STD_LOGIC;
		pwm4B					: IN	STD_LOGIC;
		pwm5A					: IN	STD_LOGIC;
		pwm5B					: IN	STD_LOGIC;
		pwm6A					: IN	STD_LOGIC;
		pwm6B					: IN	STD_LOGIC;
		EPWMSYNCI               : IN	STD_LOGIC;      -- or output
		EPWMSYNCO               : IN	STD_LOGIC;
--------------------------------------------------------------------------------
-- IO extension of dsp
		IOEN					: OUT	STD_LOGIC;
		IO_RW					: OUT	STD_LOGIC;
		EXT_IN_CE				: OUT	STD_LOGIC;
		EXT_OUT_CE				: OUT	STD_LOGIC;
--------------------------------------------------------------------------------
		Encoder_ENn				: OUT	STD_LOGIC; 
--------------------------------------------------------------------------------
        AD_CSn				    : OUT	STD_LOGIC; 
        AD_clk				    : OUT	STD_LOGIC;
        AD_EOCn					: OUT	STD_LOGIC;
        ADCRESET				: OUT	STD_LOGIC;   		
--------------------------------------------------------------------------------
--extension of CPLD
        GPIOP0				    : in	STD_LOGIC; 
        GPIOP1				    : OUT	STD_LOGIC; 
        GPIOP2				    : in	STD_LOGIC; 
        GPIOP3				    : OUT	STD_LOGIC; 
        GPIOP4				    : in	STD_LOGIC; 
        GPIOP5				    : OUT	STD_LOGIC; 
        GPIOP6				    : in	STD_LOGIC; 
        GPIOP7				    : OUT	STD_LOGIC;
        GPIOP8				    : in	STD_LOGIC; 
        GPIOP9				    : OUT	STD_LOGIC; 
        GPIOP10				    : in	STD_LOGIC; 
        GPIOP11				    : OUT	STD_LOGIC; 
        GPIOP12				    : OUT	STD_LOGIC; 
        GPIOP13				    : OUT	STD_LOGIC;  
        GPIOP14				    : OUT	STD_LOGIC;  
        GPIOP15				    : OUT	STD_LOGIC; 
-----
        PWMO1				    : buffer	STD_LOGIC;   
        PWMO2				    : OUT	STD_LOGIC;   
        PWMO3				    : OUT	STD_LOGIC;   
        PWMO4				    : OUT	STD_LOGIC;   
        PWMO5				    : OUT	STD_LOGIC;   
        PWMO6				    : OUT	STD_LOGIC;   
        PWMO7				    : OUT	STD_LOGIC;   
        PWMO8				    : OUT	STD_LOGIC;   
        PWMO9				    : OUT	STD_LOGIC;   
        PWMO10				    : OUT	STD_LOGIC;   
        PWMO11				    : OUT	STD_LOGIC;   
        PWMO12				    : OUT	STD_LOGIC;   
        PWMO13				    : OUT	STD_LOGIC;   
        PWMO14				    : OUT	STD_LOGIC;   
--------
        GPI0				    : OUT	STD_LOGIC;   
        GPI1				    : OUT	STD_LOGIC;   
        GPI2				    : OUT	STD_LOGIC;   
        GPI3				    : OUT	STD_LOGIC;   
        GPI4				    : OUT	STD_LOGIC;   
        GPI5				    : OUT	STD_LOGIC;   
        GPI6				    : OUT	STD_LOGIC;   
        GPI7				    : OUT	STD_LOGIC;   
        GPI8				    : OUT	STD_LOGIC
 );
END F28335_CPLD;
--****************************************************************************************************
ARCHITECTURE a OF F28335_CPLD IS

SIGNAL	data_inA,data_inB, data_inC, data_inU, data_inV, data_inW: std_logic_vector(15 downto 0);
SIGNAL	FB_A, FB_B, FB_C, FB_U, FB_V, FB_W:std_logic;
SIGNAL	data_outA,data_outB, data_outC, data_outU, data_outV, data_outW: std_logic_vector(5 downto 1);
SIGNAL	pulse_A, pulse_B, pulse_C, pulse_U, pulse_V, pulse_W:std_logic;
SIGNAL	pwm_A, pwm_B, pwm_C, pwm_U, pwm_V, pwm_W: std_logic_vector(3 downto 1);
SIGNAL	start_test_A, start_test_B, start_test_C: std_logic;
SIGNAL	start_test_U, start_test_V, start_test_W: std_logic;
SIGNAL	err_com, err_drv: std_logic;
SIGNAL	err_com_A, err_com_B, err_com_C, err_com_U, err_com_V, err_com_W: std_logic;
SIGNAL	err_drv_A, err_drv_B, err_drv_C, err_drv_U, err_drv_V, err_drv_W: std_logic;
SIGNAL	err_com_drv: std_logic_vector(15 downto 0);
SIGNAL	mode_u, mode_temp_u: std_logic_vector(15 downto 0);
SIGNAL	mode_v, mode_temp_v: std_logic_vector(15 downto 0);
SIGNAL	mode_w, mode_temp_w: std_logic_vector(15 downto 0);
SIGNAL	mode_a, mode_temp_a: std_logic_vector(15 downto 0);
SIGNAL	mode_b, mode_temp_b: std_logic_vector(15 downto 0);
SIGNAL	mode_c, mode_temp_c: std_logic_vector(15 downto 0);
SIGNAL	WEn_last			 :std_logic_vector(4 downto 0);
SIGNAL	DSP_lock: std_logic;
SIGNAL	count_EPWMSYNCO	:integer range 0 to 80010;
SIGNAL	EPWMSYNCO_last: std_logic;
SIGNAL	com_clk: std_logic;
CONSTANT value_com_low	:INTEGER:=20;
CONSTANT value_com		:INTEGER:=170;
SIGNAL	counter_com	:integer range 0 to value_com;
	
component Decoder
port(   reset,clk 	:in std_logic;
		clk_low		:in std_logic;
		pulse		:in std_logic;
		start_test	:out std_logic;
		err_com		:out std_logic;
		data		:buffer std_logic_vector(16	downto 1)
    );
end component;

component Encoder is
port(   reset,clk 	:in std_logic;
		start	:in std_logic;
		data	:in std_logic_vector(5	downto 1);
		pulse	:out std_logic
    );
end component;

BEGIN
-----------------------------------------------------------------------------------------------
----外设控制
	IOEN <= '0' WHEN ( ((ADD="000000000") OR (ADD="000000001") OR (ADD="000000010")) AND (ZCS0n='0')) ELSE '1';
	IO_RW <= RnW;
	EXT_IN_CE <= '0' WHEN ((ADD="000000000") AND  (ZCS0n='0') AND (RnW='1')) ELSE '1';
	EXT_OUT_CE <= '0' WHEN((ADD="000000001") AND  (ZCS0n='0') AND (RnW='0')) ELSE '1';
	Encoder_ENn <= '0' WHEN ((ADD="000000010") AND  (ZCS0n='0') AND (RnW='1')) ELSE '1';
-----------------------------------------------------------------------------------------------
----传递给DSP的IO口
	PDPINTA <= FOA;  -- output to dsp
 	PDPINTB <= FOB;  -- output to dsp			
	--OC <= OCA AND OCB;  --output to dsp
	OC<='Z';
	ECAP5 <= '0';
	ECAP6 <= '0';	
	USBCSn	<= '1'; 
-------------------------------------------------------------------------------------------------
----将从下位机得到的串行数据送到DSP
DSP_Read:process(reset, clk)
begin
if reset='0' then
	DATA <="ZZZZZZZZZZZZZZZZ";
elsif clk'event and clk='1' then
	if (RnW='1' AND ZCS0n='0') then
		case ADD is
			when "000010110" => DATA<=data_inA;
			when "000010111" => DATA<=data_inB;
			when "000011000" => DATA<=data_inC;
			when "000011001" => DATA<=data_inU;
			when "000011010" => DATA<=data_inV;
			when "000011011" => DATA<=data_inW;
			when "000100000" => DATA<=err_com_drv;
			when others => DATA <="ZZZZZZZZZZZZZZZZ";
		end case;
	else
		DATA <="ZZZZZZZZZZZZZZZZ";
	end if;
end if;
end process;
--------------------------------------------------------------------------------------------------------------------
----从DSP得到脉冲封锁以及各个开关管工作模式信息
DSP_Write:process(reset, clk)
begin
if reset='0' then
	pwmAEn<='0';
	pwmBEn<='0';
	mode_temp_a<=X"0000";
	mode_temp_b<=X"0000";
	mode_temp_c<=X"0000";
	mode_temp_u<=X"0000";
	mode_temp_v<=X"0000";
	mode_temp_w<=X"0000";
	WEn_last<="11111";
elsif clk'event and clk='1'  then
	WEn_last(4)<=WEn_last(3);
	WEn_last(3)<=WEn_last(2);
	WEn_last(2)<=WEn_last(1);
	WEn_last(1)<=WEn_last(0);
	WEn_last(0)<=WEn;
	
	if (WEn_last(3 downto 0)="1100" AND ZCS0n='0') then
		case ADD is
			when "000000101" => pwmAEn<='1';
			when "000000110" => pwmAEn<='0';
			when "000000111" => pwmBEn<='1';
			when "000001000" => pwmBEn<='0';	
			when "000010000" => mode_temp_a<=DATA;
			when "000010001" => mode_temp_b<=DATA;
			when "000010010" => mode_temp_c<=DATA;
			when "000010011" => mode_temp_u<=DATA;
			when "000010100" => mode_temp_v<=DATA;
			when "000010101" => mode_temp_w<=DATA;
			when others => NULL;
		end case;
	end if;
end if;
end process;
-------------------------------------------------------------
--检测DSP工作状态
DSP_detect: process(reset, clk)
begin
if reset='0' then
	DSP_lock<='1';
	count_EPWMSYNCO<=0;
	EPWMSYNCO_last<='0';
elsif clk'event and clk='1' then
	if EPWMSYNCO=EPWMSYNCO_last then 
		count_EPWMSYNCO<=count_EPWMSYNCO+1;
		if count_EPWMSYNCO>=80000 then
			count_EPWMSYNCO<=80000;
			DSP_lock<='1';
		else
			DSP_lock<='0';
		end if;
	else
		count_EPWMSYNCO<=0;
		DSP_lock<='0';
	end if;
	EPWMSYNCO_last<=EPWMSYNCO;
end if;
end process;	

------------------------------------------------------------------------------------------------------------------
--同步PWM并解析工作模式信号
----10H, 01L, 11PWM, 00Lock
PWM_OUT:process(EPWMSYNCO)
begin
if EPWMSYNCO'event and EPWMSYNCO='1' then
	mode_a<=mode_temp_a;
	mode_b<=mode_temp_b;
	mode_c<=mode_temp_c;
	mode_u<=mode_temp_u;
	mode_v<=mode_temp_v;
	mode_w<=mode_temp_w;
end if;
end process;

pwm_A(1)<=((mode_a(1) and mode_a(0)) and pwm1A) or (mode_a(1) and (not mode_a(0)));
pwm_A(2)<=((mode_a(3) and mode_a(2)) and pwm1A) or (mode_a(3) and (not mode_a(2)));
pwm_A(3)<=((mode_a(5) and mode_a(4)) and pwm1A) or (mode_a(5) and (not mode_a(4)));
pwm_B(1)<=((mode_b(1) and mode_b(0)) and pwm2A) or (mode_b(1) and (not mode_b(0)));
pwm_B(2)<=((mode_b(3) and mode_b(2)) and pwm2A) or (mode_b(3) and (not mode_b(2)));
pwm_B(3)<=((mode_b(5) and mode_b(4)) and pwm2A) or (mode_b(5) and (not mode_b(4)));
pwm_C(1)<=((mode_c(1) and mode_c(0)) and pwm3A) or (mode_c(1) and (not mode_c(0)));
pwm_C(2)<=((mode_c(3) and mode_c(2)) and pwm3A) or (mode_c(3) and (not mode_c(2)));
pwm_C(3)<=((mode_c(5) and mode_c(4)) and pwm3A) or (mode_c(5) and (not mode_c(4)));

pwm_U(1)<=((mode_u(1) and mode_u(0)) and pwm1B) or (mode_u(1) and (not mode_u(0)));
pwm_U(2)<=((mode_u(3) and mode_u(2)) and pwm1B) or (mode_u(3) and (not mode_u(2)));
pwm_U(3)<=((mode_u(5) and mode_u(4)) and pwm1B) or (mode_u(5) and (not mode_u(4)));
pwm_V(1)<=((mode_v(1) and mode_v(0)) and pwm2B) or (mode_v(1) and (not mode_v(0)));
pwm_V(2)<=((mode_v(3) and mode_v(2)) and pwm2B) or (mode_v(3) and (not mode_v(2)));
pwm_V(3)<=((mode_v(5) and mode_v(4)) and pwm2B) or (mode_v(5) and (not mode_v(4)));
pwm_W(1)<=((mode_w(1) and mode_w(0)) and pwm3B) or (mode_w(1) and (not mode_w(0)));
pwm_W(2)<=((mode_w(3) and mode_w(2)) and pwm3B) or (mode_w(3) and (not mode_w(2)));
pwm_W(3)<=((mode_w(5) and mode_w(4)) and pwm3B) or (mode_w(5) and (not mode_w(4)));

------------------------------------------------------------------------------------------------------------------
----接收下位机串行信号并解码
receive_A:Decoder port map
(
 pulse=>(not FB_A),
 reset=>reset,
 clk=>clk,
 clk_low=>com_clk,
 start_test=>start_test_A,
 err_com=>err_com_A,
 data=>data_inA
);

receive_B:Decoder port map
(
 pulse=>(not FB_B),
 reset=>reset,
 clk=>clk,
 clk_low=>com_clk,
 start_test=>start_test_B,
 err_com=>err_com_B,
 data=>data_inB
);

receive_C:Decoder port map
(
 pulse=>(not FB_C),
 reset=>reset,
 clk=>clk,
 clk_low=>com_clk,
 start_test=>start_test_C,
 err_com=>err_com_C,
 data=>data_inC
);

receive_U:Decoder port map
(
 pulse=>(not FB_U),
 reset=>reset,
 clk=>clk,
 clk_low=>com_clk,
 start_test=>start_test_U,
 err_com=>err_com_U,
 data=>data_inU
);

receive_V:Decoder port map
(
 pulse=>(not FB_V),
 reset=>reset,
 clk=>clk,
 clk_low=>com_clk,
 start_test=>start_test_V,
 err_com=>err_com_V,
 data=>data_inV
);

receive_W:Decoder port map
(
 pulse=>(not FB_W),
 reset=>reset,
 clk=>clk,
 clk_low=>com_clk,
 start_test=>start_test_W,
 err_com=>err_com_W,
 data=>data_inW
);


err_drv_A<=not data_inA(15);
err_drv_B<=not data_inB(15);
err_drv_C<=not data_inC(15);
err_drv_U<=not data_inU(15);
err_drv_V<=not data_inV(15);
err_drv_W<=not data_inW(15);

err_drv <= err_drv_A or err_drv_B or err_drv_C or err_drv_U or err_drv_V or err_drv_W;
err_com <= err_com_A or err_com_B or err_com_C or err_com_U or err_com_V or err_com_W;
err_com_drv<="00" & err_com_A & err_com_B & err_com_C & err_com_U & err_com_V & err_com_W & "00" & err_drv_A & err_drv_B & err_drv_C & err_drv_U & err_drv_V & err_drv_W;
----------------------------------------------------------------------------------------------------------------
----向下位机发送串行PWM信号，首位是使能信号，1使能0封锁
data_outA(5)<=pwmAEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_a(7);
data_outA(4)<=pwmAEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_a(6);
data_outA(3 downto 1)<=pwm_A;

data_outB(5)<=pwmAEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_b(7);
data_outB(4)<=pwmAEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_b(6);
data_outB(3 downto 1)<=pwm_B;

data_outC(5)<=pwmAEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_c(7);
data_outC(4)<=pwmAEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_c(6);
data_outC(3 downto 1)<=pwm_C;

data_outU(5)<=pwmBEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_u(7);
data_outU(4)<=pwmBEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_u(6);
data_outU(3 downto 1)<=pwm_U;

data_outV(5)<=pwmBEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_v(7);
data_outV(4)<=pwmBEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_v(6);
data_outV(3 downto 1)<=pwm_V;

data_outW(5)<=pwmBEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_w(7);
data_outW(4)<=pwmBEn and (not err_drv) and (not err_com) and (not DSP_lock) and mode_w(6);
data_outW(3 downto 1)<=pwm_W;

com_clk_generation:process(clk,reset)
begin
if reset='0' then
	counter_com<=0;
	com_clk<='0';
elsif clk'event and clk='1' then
	if counter_com < value_com_low then 
		com_clk<='0';
	else 
		com_clk<='1';
	end if;
	counter_com<=(counter_com+1);
	if counter_com = value_com then
			counter_com<=0 ;
	end if;
end if;
end process;

sendA:Encoder port map
(
 pulse=>pulse_A,
 reset=>reset,
 clk=>clk,
 start=>com_clk,
 data=>data_outA
);

sendB:Encoder port map
(
 pulse=>pulse_B,
 reset=>reset,
 clk=>clk,
 start=>com_clk,
 data=>data_outB
);

sendC:Encoder port map
(
 pulse=>pulse_C,
 reset=>reset,
 clk=>clk,
 start=>com_clk,
 data=>data_outC
);

sendU:Encoder port map
(
 pulse=>pulse_U,
 reset=>reset,
 clk=>clk,
 start=>com_clk,
 data=>data_outU
);

sendV:Encoder port map
(
 pulse=>pulse_V,
 reset=>reset,
 clk=>clk,
 start=>com_clk,
 data=>data_outV
);

sendW:Encoder port map
(
 pulse=>pulse_W,
 reset=>reset,
 clk=>clk,
 start=>com_clk,
 data=>data_outW
);
------------------------------------------------------------------------------------------------------------------
--指定输入输出接口
FB_A<=GPIOP0;
GPIOP1<=pulse_A;
FB_B<=GPIOP2;
GPIOP3<=pulse_B;
FB_C<=GPIOP4;
GPIOP5<=pulse_C;

FB_U<=GPIOP6;
GPIOP7<=pulse_U;
FB_V<=GPIOP8;
GPIOP9<=pulse_V;
FB_W<=GPIOP10;
GPIOP11<=pulse_W;

GPIOP12<='Z';
GPIOP13<='Z';
GPIOP14<='Z';
GPIOP15<='Z';
PWMO1<='Z';
PWMO2<='Z';
PWMO3<='Z';
PWMO4<='Z';
PWMO5<='Z';
PWMO6<='Z';
PWMO7<='Z';
PWMO8<='Z';
PWMO9<='Z';
PWMO10<='Z';
PWMO11<='Z';
PWMO12<='Z';
PWMO13<='Z';
PWMO14<='Z';
GPI0<=pwm_W(1);
GPI1<=DSP_lock;
GPI2<=pwm3B;
GPI3<=(not FB_V);
GPI4<=EPWMSYNCO;
GPI5<=(not FB_W);
GPI6<='Z';
GPI7<='Z';
GPI8<=DSP_lock;

AD_CLK<='Z';
AD_EOCn<='Z';
ADCRESET<='Z';
AD_CSn<='Z';
---------------------------------------------------------------------------------
END a;


