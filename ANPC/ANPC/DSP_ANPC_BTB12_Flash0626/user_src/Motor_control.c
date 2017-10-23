
#include "Motor_control.h"
// Prototype statements for functions found within this file.
#define startCpuTimer0() CpuTimer0Regs.TCR.bit.TSS=0
interrupt void cpu_timer0_isr(void);
interrupt void ADC_conversion(void);
interrupt void TZ_int(void);
interrupt void qep1_int(void);
//interrupt void cpu_timer1_isr(void);
//interrupt void cpu_timer2_isr(void);
void scib_echoback_init(void);
void scib_fifo_init(void);
//int level_calc(float* usin);
extern void scib_tx_isr(void);
extern void scib_rx_isr(void);
extern void InitXintf(void);
extern void InitEPwmGpio(void);
extern void InitEPwmSyncGpio(void);

extern unsigned int RamfuncsLoadStart;
extern unsigned int RamfuncsLoadEnd;
extern unsigned int RamfuncsRunStart;

//=========================================================
extern void alarm_Normal(void);
extern void alarm_Hardware(void);
extern void alarm_Software(void);
extern void alarm_PWMdisable(void);
extern void alarm_brake(void);

extern eQEP qep1;
extern FE fe1;
extern PI_SPEED pi_speed;
extern PI_fun pi_isd;
extern PI_fun pi_isq;

int LED_count=0;
int LED_count2=0;
int LED_num=0;

int Ext_Ctrl=0;
//=======================================================================================//
int prechargeA=0;
int prechargeB=0;
int prechargeC=0;

int prechargeU=0;
int prechargeV=0;
int prechargeW=0;
int prechargeDC=0;

float precharge_duty=0;
//=======================================================================================//
//	for internal AD converter
	int AD_chan0 = 0;
	int AD_chan1 = 0;
	int AD_chan2 = 0;
	int AD_chan3 = 0;
	int AD_chan4 = 0;
	int AD_chan5 = 0;
	int AD_chan6 = 0;
	int AD_chan7 = 0;

	int AD_chan8 = 0;
	int AD_chan9 = 0;
	int AD_chan10 = 0;
	int AD_chan11 = 0;
	int AD_chan12 = 0;
	int AD_chan13 = 0;
	int AD_chan14 = 0;
	int AD_chan15 = 0;
//=======================================================================================//
	para_motor ParaMotor = para_motor_DEFAULTS;
	para_converter ParaConvt = para_converter_DEFAULTS;
	VC_Grid VcGrid = VC_Grid_DEFAULTS;
	Vector_ctrl VcMotor = Vector_ctrl_DEFAULTS;
	PWM_Update pwm1 = PWM_Update_DEFAULTS;
//==================================================================================//
//	for ANPC
	phase_state phase_a = phase_state_DEFAULTS;
	phase_state phase_b = phase_state_DEFAULTS;
	phase_state phase_c = phase_state_DEFAULTS;

	phase_state phase_u = phase_state_DEFAULTS;
	phase_state phase_v = phase_state_DEFAULTS;
	phase_state phase_w = phase_state_DEFAULTS;

	ZERO_SEQUENCE zero_seq_volt = ZERO_SEQUENCE_DEFAULTS;

	float udc1,udc2, udc, ufc_ref, udc_ref;
	float ufa,ufb,ufc,ufu,ufv,ufw;
	float ia=0, ib=0, ic=0, iu=0, iv=0, iw=0;
	float uab, ubc, uca, usa, usb, usc;
	float speed_ref=0;
	float iq_ref_grid=0;
	int test1,test2;

	LPF lpf_ia = LPF_DEFAULTS;
	LPF lpf_ib = LPF_DEFAULTS;
	LPF lpf_ic = LPF_DEFAULTS;
	LPF lpf_iu = LPF_DEFAULTS;
	LPF lpf_iv = LPF_DEFAULTS;
	LPF lpf_iw = LPF_DEFAULTS;
	LPF lpf_uab = LPF_DEFAULTS;
	LPF lpf_ubc = LPF_DEFAULTS;

	float ia_lpf = 0;
	float ib_lpf = 0;
	float ic_lpf = 0;
	float iu_lpf = 0;
	float iv_lpf = 0;
	float iw_lpf = 0;
	float uab_lpf = 0;
	float ubc_lpf = 0;
	float uca_lpf = 0;

	float uoa_ref, uob_ref,uoc_ref, uou_ref, uov_ref,uow_ref;
	float uoa, uob, uoc, uou, uov, uow;
//	float m=1;
//	float uz=0;
//===================================================================================//
//	float k=1;
	int pwmdisable=0;
	int err_com_drv=0;
	int errcom=0;
	int errdrv=0;

	int OverVolDC=0;
	int OverVolFCU=0;
	int OverVolFCV=0;
	int OverVolFCW=0;
	int OverVolFCA=0;
	int OverVolFCB=0;
	int OverVolFCC=0;
	int OverVolFC=0;
	int OverCur=0;

	Uint16 err_log[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_udc1[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_udc2[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_uca[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ucb[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ucc[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ucu[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ucv[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ucw[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ia[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ib[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_ic[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_iu[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_iv[10]={0,0,0,0,0,0,0,0,0,0};
	float err_log_iw[10]={0,0,0,0,0,0,0,0,0,0};
	int err_first=10;
	int err_count=0;
//=======================================================================================//
	int scope_wave[50];
	int	scope_index0=0;
	int	scope_index1=1;
	int	scope_index2=2;
	int	scope_index3=3;
//=======================================================================================//
//=======================================================================================//
void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
	InitSysCtrl();
//	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);///////////////////////////////////
//	InitFlash();////////////////////////////////////////////


	speed_ref = 5;
	udc_ref = 620;
	precharge_duty=0.1;

	ParaMotor.Lm = 0.107;
	ParaMotor.Ls = 0.12;
	ParaMotor.Lr = 0.122;
	ParaMotor.Rs = 0474;
	ParaMotor.Rr = 0.761;
	ParaMotor.pole = 2;
	ParaMotor.J_motor = 0.002;
	ParaMotor.tao_r = ParaMotor.Lr/ParaMotor.Rr;
	ParaMotor.sigma = 1-ParaMotor.Lm*ParaMotor.Lm/ParaMotor.Ls/ParaMotor.Lr;
	ParaMotor.flux_ref = 1;
	ParaMotor.IN = 5;
	ParaMotor.PN = 2.2;
	ParaMotor.UL = 380;
	ParaMotor.FN = 50;
	ParaMotor.torq = ParaMotor.PN/DPI/ParaMotor.FN*ParaMotor.pole;

	ParaConvt.Cd = 0.0047;
	ParaConvt.Cf = 0.0066;
	ParaConvt.fs = 2000;
	ParaConvt.Tp = 1/ParaConvt.fs;
	ParaConvt.Imax = 10;
	ParaConvt.Ucfmax = 100;
	ParaConvt.Udc = 300;
	ParaConvt.Udmax = 400;
	ParaConvt.encoder_linear = 1000;

	pwm1.PWM_period=150000000/ParaConvt.fs/2;

//	qep1.Ts = 0.0025 ;//para.Ts/2.0;//4k
//	qep1.Ts_speed = 0.025;//qep1.Ts;
//	qep1.pole_pair = ParaMotor.pole;
//	qep1.encoder_linear = ParaConvt.encoder_linear;
//	qep1.qep_spd_cnt = 10;
//	EQep1Regs.QUPRD = qep1.Ts*150000000;// 40us

//	vc_motor.speed_ref = speed_ref;
	lpf_uab.wc = 1;
	lpf_uab.Tc = ParaConvt.Tp;
	lpf_ubc.wc = 1;
	lpf_ubc.Tc = ParaConvt.Tp;

	lpf_ia.wc = 1;
	lpf_ia.Tc = ParaConvt.Tp;
	lpf_ib.wc = 1;
	lpf_ib.Tc = ParaConvt.Tp;
	lpf_ic.wc = 1;
	lpf_ic.Tc = ParaConvt.Tp;
	
	lpf_iu.wc = 1;
	lpf_iu.Tc = ParaConvt.Tp;
	lpf_iv.wc = 1;
	lpf_iv.Tc = ParaConvt.Tp;
	lpf_iw.wc = 1;
	lpf_iw.Tc = ParaConvt.Tp;

//	SpeedPI.kp = 0.3;
//	SpeedPI.ki = 2;
//	SpeedPI.Ts = ParaConvt.Tp;
//	SpeedPI.max = 10;

//	IsdPI.kp = 1;
//	IsdPI.ki = 1;
//	IsdPI.Ts = ParaConvt.Tp;
//	IsdPI.max = 50;

//	IsqPI.kp = 1;
//	IsqPI.ki = 1;
//	IsqPI.Ts = ParaConvt.Tp;
//	IsqPI.max = 50;

	// for VVVF control
//	rc1.rmp_delay_cntl = 0;
//	rc1.rmp_dly_max = 100;//every 100 cycle, 10ms
//	rc1.rmp_hi_limit = 100; //limited to 100Hz
//	rc1.rmp_lo_limit = -100; 
//	rc1.setpt_value = 1.0;
//	rc1.step_freq = 0.1;// step frequency for the acceleration
	//-----------------------------------------------------//
//	rg1.base_volt = 220*1.414*0.02;//220
//	rg1.rmp_gain = 1.0;
//	rg1.Ts = ParaConvt.Tp;




// Step 2. Initalize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example
   InitXintf();
//   InitXintf16Gpio();	//zq
//	InitGpio();
//=======================================================================================//

//   
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();
//===============================================================================================//
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.TINT0 = &cpu_timer0_isr;
   //PieVectTable.XINT13 = &cpu_timer1_isr;
   //PieVectTable.TINT2 = &cpu_timer2_isr;
//   PieVectTable.SCIRXINTA=&scia_rx_isr;
//   PieVectTable.SCITXINTA=&scia_tx_isr;
	PieVectTable.SCIRXINTB=&scib_rx_isr;
    PieVectTable.SCITXINTB=&scib_tx_isr;
//   PieVectTable.ADCINT = &ADC_conversion;
   PieVectTable.SEQ1INT = &ADC_conversion;
   PieVectTable.EPWM1_TZINT = &TZ_int;
//   PieVectTable.EQEP1_INT = &qep1_int;      //*****************************

   EDIS;    // This is needed to disable write to EALLOW protected registers
//===============================================================================================//
// Step 4. Initialize the Device Peripheral. This function can be
//         found in DSP2833x_CpuTimers.c
   InitCpuTimers();   // For this example, only initialize the Cpu Timers

#if (CPU_FRQ_150MHZ)
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 150MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 150, 1000000);
   //ConfigCpuTimer(&CpuTimer1, 150, 1000000);
   //ConfigCpuTimer(&CpuTimer2, 150, 1000000);
#endif

#if (CPU_FRQ_100MHZ)
// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
// 100MHz CPU Freq, 1 second Period (in uSeconds)

   ConfigCpuTimer(&CpuTimer0, 100, 1000000);
   //ConfigCpuTimer(&CpuTimer1, 100, 1000000);
   //ConfigCpuTimer(&CpuTimer2, 100, 1000000);
#endif
// To ensure precise timing, use write-only instructions to write to the entire register. Therefore, if any
// of the configuration bits are changed in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
// below settings must also be updated.

   //CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
   //CpuTimer1Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
   //CpuTimer2Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
//===============================================================================================//
	// Step 5. User specific code, enable interrupts:
	CpuTimer0Regs.PRD.all=0x3A98;//15000   100us
	CpuTimer0Regs.TPR.all=0;
	CpuTimer0Regs.TIM.all=0;
	CpuTimer0Regs.TPRH.all=0;
	CpuTimer0Regs.TCR.bit.TSS=1;
	CpuTimer0Regs.TCR.bit.SOFT=1;
	CpuTimer0Regs.TCR.bit.FREE=1;
	CpuTimer0Regs.TCR.bit.TRB=1;
	CpuTimer0Regs.TCR.bit.TIE=1;
	CpuTimer0.InterruptCount=0;
	startCpuTimer0();

// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
// which is connected to CPU-Timer 1, and CPU int 14, which is connected
// to CPU-Timer 2:

//===============================================================================================//
	InitScibGpio();
    scib_fifo_init();	   // Initialize the SCI FIFO
    scib_echoback_init();  // Initalize SCI for echoback
    SCOPEINITNOLOOP();	
//===============================================================================================//
//--------------------------------------------------------------------------------------------//
//configure for the EV module
	InitEPwmGpio();
	InitEPwmSyncGpio();
 	InitAdc();         // For this example, init the ADC
	InitEQep1Gpio();
	Personal_set(&ParaConvt);
//***********************************
	qep1.Init(&qep1);
	VcGrid.init(&VcGrid);
	VcMotor.init(&VcMotor);

//--------------------------------------------------------------------------//
   AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0xF;  // convert and store in 8 results registers

   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;
   AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;
   AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3;

   AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4;
   AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5;
   AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x6;
   AdcRegs.ADCCHSELSEQ2.bit.CONV07 = 0x7;

   AdcRegs.ADCCHSELSEQ3.bit.CONV08 = 0x8;
   AdcRegs.ADCCHSELSEQ3.bit.CONV09 = 0x9;
   AdcRegs.ADCCHSELSEQ3.bit.CONV10 = 0xa;
   AdcRegs.ADCCHSELSEQ3.bit.CONV11 = 0xb;

   AdcRegs.ADCCHSELSEQ4.bit.CONV12 = 0xc;
   AdcRegs.ADCCHSELSEQ4.bit.CONV13 = 0xd;
   AdcRegs.ADCCHSELSEQ4.bit.CONV14 = 0xe;
   AdcRegs.ADCCHSELSEQ4.bit.CONV15 = 0xf;

   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0x1;
   AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0x1;
//============================================================================================//
	EPwm1Regs.CMPA.half.CMPA = 3750;// all is for the HRPWM extension
//---------------------------------------//
	EPwm1Regs.ETSEL.bit.SOCAEN = 1;
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
 	EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;
//===============================================================================================//

   IER |= M_INT1;// for the cput time0 interrupt, ADC interrupt
   IER |= M_INT9; // for the scia interrup
   IER |= M_INT2; // for the int of TZ of PWM
//   IER |= M_INT5; // for the eQEP int        ***************************************************
   //IER |= M_INT13;
   //IER |= M_INT14;

// Enable TINT0 in the PIE: Group 1 interrupt 7
   PieCtrlRegs.PIEIER1.bit.INTx7 = 1;// for the interrupt of cpu timer0
//-------------------------------------------------------------------------------------//
//   PieCtrlRegs.PIEIER9.bit.INTx1 = 1;// for the scia
//   PieCtrlRegs.PIEIER9.bit.INTx2 = 1; // for the scia
   PieCtrlRegs.PIEIER9.bit.INTx3 = 1;// for the scib
   PieCtrlRegs.PIEIER9.bit.INTx4 = 1; // for the scib   
//   PieCtrlRegs.PIEIER5.bit.INTx1 = 1; // for the eQEP1 int***************************
//-------------------------------------------------------------------------------------//

//   PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;// for the ADC interrupt
   PieCtrlRegs.PIEIER2.bit.INTx1 = 1;// for the interrupt of TZ
//-------------------------------------------------------------------------------------//
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM
//-------------------------------------------------------------------------------------//
	//	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1; // set GPIO16 as low level for the short 

	//	GpioDataRegs.GPASET.bit.GPIO16 = 1;
	//	GpioDataRegs.GPASET.bit.GPIO17 = 1;
	GpioDataRegs.GPADAT.bit.GPIO16 =0;   //Short
	GpioDataRegs.GPADAT.bit.GPIO17 =1;    //Brake
//-------------------------------------------------------------------------------------//
// just for test
	EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;  // Configure GPIO12 as GPIO  TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;  // Configure GPIO13 as GPIO  TZ2
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 0; // as input
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 0; // as input
//	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;// as IO  ECAP1
//	GpioCtrlRegs.GPBDIR.bit.GPIO34 =0;  // as input
	EDIS;

	*PWMa_disable = 1;
	*PWMb_disable = 1;

/*
	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0; 
	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;
	EDIS;*/
//-------------------------------------------------------------------------------------//

// Step 6. IDLE loop. Just sit and loop forever (optional):

	for(;;)
		{

		}
}


#pragma CODE_SECTION(ADC_conversion, "ramfuncs")
//==========================================================================================================================================//
interrupt void ADC_conversion(void)
{
//从CPLD读取悬浮电容电压
//分控板AD采样悬浮电容电压的凳：
// V/24k（采样电阻）*2.5（霍尔变比）*100（采样电阻）*1024/2（10位AD2V参考源）
//0.1875V对应1
	ufa=((*cpld_Ufa)&0x03ff)*0.1875-1;
	ufb=((*cpld_Ufb)&0x03ff)*0.1875-1;
	ufc=((*cpld_Ufc)&0x03ff)*0.1875-1;
	ufu=((*cpld_Ufu)&0x03ff)*0.1875-2;
	ufv=((*cpld_Ufv)&0x03ff)*0.1875-2;
	ufw=((*cpld_Ufw)&0x03ff)*0.1875-2;
//	test1 = *cpld_Ufa;
//	test2 = *cpld_Ufb;
//==============================================================//
//	从内部AD采样输入电压、电流以及母线电压、输出电流信号
//DSP内部AD采样母线电压的系数为：
// V/48k（采样电阻）*2.5（霍尔变比）*62（采样电阻）*4096/3（12位AD3V参考源）
//0.22681V对应1

//DSP内部AD采样相电流的系数为：
// 读数为：(I/1000（霍尔变比）*62（采样电阻）+3.3V)/2*4096/3（12位AD3V参考源）
//ia = (AD_chan0/4096.0*3-1.665)*2*10;
	//0.02363(0.0146484375)A对应1
	AD_chan0 = AdcRegs.ADCRESULT0>>4;
	AD_chan1 = AdcRegs.ADCRESULT1>>4;
	AD_chan2 = AdcRegs.ADCRESULT2>>4;
	AD_chan3 = AdcRegs.ADCRESULT3>>4;
	AD_chan4 = AdcRegs.ADCRESULT4>>4;
	AD_chan5 = AdcRegs.ADCRESULT5>>4;
	AD_chan6 = AdcRegs.ADCRESULT6>>4;
	AD_chan7 = AdcRegs.ADCRESULT7>>4;
	AD_chan8 = AdcRegs.ADCRESULT8>>4;
	AD_chan9 = AdcRegs.ADCRESULT9>>4;
	AD_chan10 = AdcRegs.ADCRESULT10>>4;
	AD_chan11 = AdcRegs.ADCRESULT11>>4;
//DSP板上接口从左至右依次是chan4, 3, 2, 1, 0, 8, 9, 10, 11
//	asm("NOP");

	udc1 = AD_chan11*0.22681-1;
	udc2 = AD_chan10*0.22681-2;
	udc = udc1+udc2;
	ufc_ref = udc/4;

	iu = (AD_chan9-2178)*0.02363;
	iw = (AD_chan8-2175)*0.02363;
	iv = -iu-iw;

	ia = (AD_chan0-2173)*0.02363;
	ic = (AD_chan1-2174)*0.02363;
	ib = -ia-ic;

	uab = (AD_chan2-2165)*0.4536;
	ubc = (AD_chan3-2166)*0.4536;
	uca = -uab-ubc;
//===================================================================//
//更新每相桥臂的电压电流信息
	phase_a.ufc = ufa;
	phase_b.ufc = ufb;
	phase_c.ufc = ufc;

	phase_u.ufc = ufu;
	phase_v.ufc = ufv;
	phase_w.ufc = ufw;

	phase_a.udc1 = udc1;
	phase_a.udc2 = udc2;
	phase_a.ufc_ref = ufc_ref;
	phase_b.udc1 = udc1;
	phase_b.udc2 = udc2;
	phase_b.ufc_ref = ufc_ref;
	phase_c.udc1 = udc1;
	phase_c.udc2 = udc2;
	phase_c.ufc_ref = ufc_ref;

	phase_u.udc1 = udc1;
	phase_u.udc2 = udc2;
	phase_u.ufc_ref = ufc_ref;
	phase_v.udc1 = udc1;
	phase_v.udc2 = udc2;
	phase_v.ufc_ref = ufc_ref;
	phase_w.udc1 = udc1;
	phase_w.udc2 = udc2;
	phase_w.ufc_ref = ufc_ref;
//============================================================
//对三相电流进行滤波，消除测量带来的直流偏置
	lpf_iu.x = iu;
	lpf_iu.calc(&lpf_iu);
	lpf_iv.x = iv;
	lpf_iv.calc(&lpf_iv);
	lpf_iw.x = iw;
	lpf_iw.calc(&lpf_iw);

	iu_lpf = lpf_iu.x - lpf_iu.y;
	iv_lpf = lpf_iv.x - lpf_iv.y;
	iw_lpf = -iu_lpf-iv_lpf;

	lpf_ia.x = ia;
	lpf_ia.calc(&lpf_ia);
	lpf_ib.x = ib;
	lpf_ib.calc(&lpf_ib);
	lpf_ic.x = ic;
	lpf_ic.calc(&lpf_ic);

	ia_lpf = lpf_ia.x - lpf_ia.y;
	ib_lpf = lpf_ib.x - lpf_ib.y;
	ic_lpf = -ia_lpf-ib_lpf;

	lpf_uab.x = uab;
	lpf_uab.calc(&lpf_uab);
	lpf_ubc.x = ubc;
	lpf_ubc.calc(&lpf_ubc);

	uab_lpf = uab - lpf_uab.y;
	ubc_lpf = ubc - lpf_ubc.y;
	uca_lpf = -uab_lpf-ubc_lpf;

	phase_a.i = ia_lpf;
	phase_b.i = ib_lpf;
	phase_c.i = ic_lpf;

	phase_u.i = iu;
	phase_v.i = iv;
	phase_w.i = iw;

//==================================================================//
//从CPLD读取通信和驱动故障状态，全0表示无故障，否则有故障
	err_com_drv=*err_cpld;

//通信故障保护,0为无故障
//	if((err_com_drv & 0x0F00)==0)
	if((err_com_drv & 0xFF00)==0)
		errcom=0;
	else 
		errcom=1;
//驱动故障保护,0为无故障
//	if((err_com_drv & 0x000F)==0)
	if((err_com_drv & 0x00FF)==0)
		errdrv=0;
	else 
		errdrv=1;
//桥臂过流保护，0为未过流
	if ((ia_lpf<20)&&(ib_lpf<20)&&(ic_lpf<20)&&(iu_lpf<20)&&(iv_lpf<20)&&(iw_lpf<20))
//	if ((abs(iu)<10)&&(abs(iv)<10)&&(abs(iw)<10))
		OverCur=0;
	else
		OverCur=1;
//母线电容过压保护，0为未过压
	if ((udc1<380)&&(udc2<380))
		OverVolDC=0;
	else
		OverVolDC=1;
//悬浮电容过压保护，0为未过压
	if (ufa<200)
		OverVolFCA=0;
	else
		OverVolFCA=1;

	if (ufb<200)
		OverVolFCB=0;
	else
		OverVolFCB=1;

	if (ufc<200)
		OverVolFCC=0;
	else
		OverVolFCC=1;

	if (ufu<200)
		OverVolFCU=0;
	else
		OverVolFCU=1;

	if (ufv<200)
		OverVolFCV=0;
	else
		OverVolFCV=1;

	if (ufw<200)
		OverVolFCW=0;
	else
		OverVolFCW=1;

	if((OverVolFCA==0)&&(OverVolFCB==0)&&(OverVolFCC==0)&&(OverVolFCU==0)&&(OverVolFCV==0)&&(OverVolFCW==0))
		OverVolFC=0;
	else
		OverVolFC=1;
//======================================================================//
//LED显示，指示当前运行状态
	LED_count++;
	if((errcom ==1)||(errdrv ==1))
		alarm_Hardware();		//双闪
	else if((OverCur==1)||(OverVolDC==1)||(OverVolFC==1))
		alarm_Software();			//上下轮流闪
	else if(pwmdisable ==1)
		alarm_PWMdisable();			//左右轮流闪
	else
		alarm_Normal();

//===================================================================================================================================//
//检测状态，预备加主电，开始预充电
//开关模式信号：10代表模式1, 01代表模式0, 11代表PWM模式, 00代表全封锁脉冲。
//0~1位控制T1~T4,2~3位控制T5~T6，4~5位控制T7~T8
if(((*ext_in)&0x0001) == 0x0000)
{
	Ext_Ctrl = 0;	//封锁所有外部接触器等待指示
	if((errcom==0)&&(errdrv==0)&&(OverCur==0)&&(OverVolDC==0)&&(OverVolFC==0))
	{
		pwmdisable  = 0;
		*PWMa_enable = 1;
		*PWMb_enable = 1;

		*PWMA_mode = 0x0000;		//00 00 00 00 00 00 00 00
		*PWMB_mode = 0x0000;
		*PWMC_mode = 0x0000;

//		*PWMU_mode = 0x0000;		//00 00 00 00 00 00 00 00
//		*PWMV_mode = 0x0000;
//		*PWMW_mode = 0x0000;
	}
	else
	{
		pwmdisable =1;
		*PWMa_disable = 1;
		*PWMb_disable = 1;
	}

	if (((*ext_in)&0x00FF) == 0x0000)
	{
		prechargeA=0;
		prechargeB=0;
		prechargeC=0;

		prechargeU=0;
		prechargeV=0;
		prechargeW=0;
		prechargeDC=0;

		*PWMU_mode = 0x0000;		//00 00 00 00 00 00 00 00
		*PWMV_mode = 0x0000;
		*PWMW_mode = 0x0000;

		speed_ref = 5;
		VcMotor.reset(&VcMotor);
		VcGrid.init(&VcGrid);
		VcMotor.init(&VcMotor);
	}


}

//=====================================================================================================================================//
//*********************直流母线预充电*********************
if(((*ext_in)&0x00FF) == 0x0001)
{
//	if((errcom==1)||(errdrv==1)||(OverCur==1)||(OverVolDC==1)||(OverVolFC==1))
	if((errcom==1)||(errdrv==1)||(OverVolFC==1))

	{
		*PWMa_disable = 1;
		*PWMb_disable = 1;
		pwmdisable =1;
	}

	if ((prechargeA==1)&&(prechargeB==1)&&(prechargeC==1)&&(prechargeDC==1))
	{
		Ext_Ctrl = 0x01;//预充电完毕，合上主接触器
		*PWMA_mode = 0x0000;		//00 00 00 00 00 00 00 00
		*PWMB_mode = 0x0000;
		*PWMC_mode = 0x0000;

		*PWMU_mode = 0x0000;		//00 00 00 00 00 00 00 00
		*PWMV_mode = 0x0000;
		*PWMW_mode = 0x0000;
	}
	else
	{
		Ext_Ctrl = 0x02;	//合上预充电接触器，开始直流母线预充电
	}

	if (prechargeA==0)
	{
		*PWMA_mode = 0x009A;  //10 01 10 10 
	}
	if (prechargeB==0)
	{
		*PWMB_mode = 0x009A;  //10 01 10 10 
	} 
	if (prechargeC==0)
	{
		*PWMC_mode = 0x009A;  //10 01 10 10 
	}

	if(phase_a.ufc>udc_ref/4*1.1)
	{
		prechargeA=1;
		*PWMA_mode = 0x0000;
	}

	if(phase_b.ufc>udc_ref/4*1.1)
	{
		prechargeB=1;
		*PWMB_mode = 0x0000;
	}

	if(phase_c.ufc>udc_ref/4*1.1)
	{
		prechargeC=1;
		*PWMC_mode = 0x0000;
	} 

	if (udc>udc_ref*0.8)
	{
		prechargeDC=1;
	}
}
else
{
//================================================================================================================================//
	if((errcom==1)||(errdrv==1)||(OverCur==1)||(OverVolDC==1)||(OverVolFC==1))
	{
		err_count+=1;
		if (err_count>=3)
		{
			pwmdisable =1;
			*PWMa_disable = 1;
			*PWMb_disable = 1;
		}

		if (err_first==10)
			err_first=0;
	}

	if (err_first<10)
	{
		err_log[err_first] = (errcom<<6) + (errdrv<<5) + (OverCur<<4) + (OverVolDC<<3) + (OverVolFCU<<2) + (OverVolFCV<<1) + OverVolFCW;
		err_log_udc1[err_first] = udc1;
		err_log_udc2[err_first] = udc2;
		err_log_uca[err_first] = ufa;
		err_log_ucb[err_first] = ufb;
		err_log_ucc[err_first] = ufc;
		err_log_ucu[err_first] = ufu;
		err_log_ucv[err_first] = ufv;
		err_log_ucw[err_first] = ufw;
		err_log_ia[err_first] = ia;
		err_log_ib[err_first] = ib;
		err_log_ic[err_first] = ic;
		err_log_iu[err_first] = iu;
		err_log_iv[err_first] = iv;
		err_log_iw[err_first] = iw;

		err_first+=1;
	} 

  if ((((*ext_in)&0x0003) == 0x0003)&&(prechargeDC==1))	//*********************拨码开关第二位控制整流器工作*********************
  {
	Ext_Ctrl = 0x01;	//确认断开预充电接触器并合上主接触器

	VcGrid.isa = -ia_lpf;
	VcGrid.isb = -ib_lpf;
	VcGrid.isc = -ic_lpf;
	VcGrid.udc = udc;
	VcGrid.uab = uab_lpf;
	VcGrid.ubc = ubc_lpf;
	VcGrid.uca = uca_lpf;
	VcGrid.udc_ref = udc_ref;
	VcGrid.iq_ref  = iq_ref_grid;

	VcGrid.calc(&VcGrid);
	
	uoa_ref = VcGrid.ua_ref/ufc_ref;
	uob_ref = VcGrid.ub_ref/ufc_ref;
	uoc_ref = VcGrid.uc_ref/ufc_ref;

	if (uoa_ref>2.3)
		uoa_ref = 2.3;
	if (uoa_ref<-2.3)
		uoa_ref = -2.3;
	if (uob_ref>2.3)
		uob_ref = 2.3;
	if (uob_ref<-2.3)
		uob_ref = -2.3;
	if (uoc_ref>2.3)
		uoc_ref = 2.3;
	if (uoc_ref<-2.3)
		uoc_ref = -2.3;
//==================================================================//
//中点电压平衡控制算法，选择最优零序电压注入，得到新的三相电压参考值
	phase_a.FC_balancing(&phase_a);
	phase_b.FC_balancing(&phase_b);
	phase_c.FC_balancing(&phase_c);
/*
	zero_seq_volt.uin[0] = uoa_ref;
	zero_seq_volt.uin[1] = uob_ref;
	zero_seq_volt.uin[2] = uoc_ref;
	zero_seq_volt.io[0] = phase_a.i;
	zero_seq_volt.io[1] = phase_b.i;
	zero_seq_volt.io[2] = phase_c.i;
	zero_seq_volt.redundant[0] = phase_a.redundant;
	zero_seq_volt.redundant[1] = phase_b.redundant;
	zero_seq_volt.redundant[2] = phase_c.redundant;
	zero_seq_volt.Inp_ref = ParaConvt.Cd*(udc2-udc1)/ParaConvt.Tp;
	zero_seq_volt.order_volt(&zero_seq_volt);
	zero_seq_volt.uz_calc(&zero_seq_volt);


	uoa = 2+zero_seq_volt.uout[0];
	uob = 2+zero_seq_volt.uout[1];
	uoc = 2+zero_seq_volt.uout[2];

	uoa = 2+uoa_ref;
	uob = 2+uob_ref;
	uoc = 2+uoc_ref;
*/

	uoa = 2+uoa_ref+(-zero_seq_volt.umin0-zero_seq_volt.umax0)/2;
	uob = 2+uob_ref+(-zero_seq_volt.umin0-zero_seq_volt.umax0)/2;
	uoc = 2+uoc_ref+(-zero_seq_volt.umin0-zero_seq_volt.umax0)/2;

//===================================================================//
//根据三相参考电压计算层级、占空比，发出PWM信号	
	phase_a.usin = uoa;
	phase_b.usin = uob;
	phase_c.usin = uoc;
	
	phase_a.level_calc(&phase_a);
	phase_b.level_calc(&phase_b);
	phase_c.level_calc(&phase_c);

	phase_a.PWM_mode_calc(&phase_a);
	phase_b.PWM_mode_calc(&phase_b);
	phase_c.PWM_mode_calc(&phase_c);

	*PWMA_mode = phase_a.PWM_mode;
	*PWMB_mode = phase_b.PWM_mode;
	*PWMC_mode = phase_c.PWM_mode;

	pwm1.Ta = 1 - phase_a.duty;
	pwm1.Tb = 1 - phase_b.duty;
	pwm1.Tc = 1 - phase_c.duty;

	pwm1.Update(&pwm1);
  }
  else
  {
	*PWMA_mode = 0x0000;	
	*PWMB_mode = 0x0000;	
	*PWMC_mode = 0x0000;
  }

//==============================================================================================================================//
  if((((*ext_in)&0x0080) == 0x0080)&&(prechargeDC==1))		//*********************拨码开关第8位控制逆变器工作*********************
//  if(((*ext_in)&0x0080) == 0x0080)
  {

	Ext_Ctrl &= 0xFD;	//确认断开预充电接触器

	if((prechargeU==0)||(prechargeV==0)||(prechargeW==0))
	{
		//悬浮电容预充电
		phase_u.duty=precharge_duty;
		phase_v.duty=precharge_duty;
		phase_w.duty=precharge_duty;

		if(phase_u.ufc>phase_u.ufc_ref*1.1)
		{
			prechargeU=1;
		}

		if(phase_v.ufc>phase_v.ufc_ref*1.07)
		{
			prechargeV=1;
		}

		if(phase_w.ufc>phase_w.ufc_ref*1.04)
		{
			prechargeW=1;
		}

		if (prechargeU==0)
		{
			if(abs(phase_u.i)<5)
			{
				*PWMU_mode = 0x0065;		//01 10 01 01
				*PWMV_mode = 0x0000;		//00 00 00 00
				*PWMW_mode = 0x00BF;		//10 11 11 11 
			}
			else
			{
				*PWMU_mode = 0x0065;		//01 00 01 01
				*PWMV_mode = 0x0000;		//00 00 00 00
				*PWMW_mode = 0x0000;		//11 01 10 10 
			}
		}
		else if(prechargeV==0)
		{
			if(abs(phase_v.i)<5)
			{
				*PWMU_mode = 0x00BF;		//10 11 11 11 
				*PWMV_mode = 0x0065;		//01 00 01 01
				*PWMW_mode = 0x0000;		//00 00 00 00
			}
			else
			{
				*PWMU_mode = 0x0000;		//00 00 00 00 
				*PWMV_mode = 0x0065;		//01 00 01 10
				*PWMW_mode = 0x0000;		//00 00 00 00
			}
		}
		else if (prechargeW==0)
		{
			if(abs(phase_w.i)<5)
			{
				*PWMU_mode = 0x0000;		//00 00 00 00
				*PWMV_mode = 0x00BF;		//10 11 11 11 
				*PWMW_mode = 0x0065;		//01 00 01 01
			}
			else
			{
				*PWMU_mode = 0x0000;		//00 00 00 00
				*PWMV_mode = 0x0000;		//11 01 10 10 
				*PWMW_mode = 0x0065;		//01 00 01 01
			}
		}
		else
		{
			*PWMU_mode = 0x0000;	
			*PWMV_mode = 0x0000;	
			*PWMW_mode = 0x0000;
		}

		pwm1.Tu = 1 - phase_u.duty;
		pwm1.Tv = 1 - phase_v.duty;
		pwm1.Tw = 1 - phase_w.duty;
		pwm1.Update(&pwm1);
	}
	else    //**********************开始正常工作*********************
  	{
		VcMotor.control_method = 1;  			// VC	
		VcMotor.speed_ref_Hz = speed_ref;
		VcMotor.wr_meca_SI = qep1.w_meca;
		VcMotor.posi_meca = qep1.posi_meca;
		VcMotor.Isa_SI = phase_u.i;				// 实际值
		VcMotor.Isb_SI = phase_v.i;
		VcMotor.Isc_SI = phase_w.i;
		VcMotor.calc(&VcMotor);

		uou_ref = VcMotor.Ua_ref/ufc_ref;
		uov_ref = VcMotor.Ub_ref/ufc_ref;
		uow_ref = VcMotor.Uc_ref/ufc_ref;

		if (uou_ref>2.3)
			uou_ref = 2.3;
		if (uou_ref<-2.3)
			uou_ref = -2.3;
		if (uov_ref>2.3)
			uov_ref = 2.3;
		if (uov_ref<-2.3)
			uov_ref = -2.3;
		if (uow_ref>2.3)
			uow_ref = 2.3;
		if (uow_ref<-2.3)
			uow_ref = -2.3;
	//======================================================================//
	//中点电压平衡控制算法，选择最优零序电压注入，得到新的三相电压参考值
		phase_u.FC_balancing(&phase_u);
		phase_v.FC_balancing(&phase_v);
		phase_w.FC_balancing(&phase_w);

		zero_seq_volt.uin[0] = uou_ref;
		zero_seq_volt.uin[1] = uov_ref;
		zero_seq_volt.uin[2] = uow_ref;
		zero_seq_volt.io[0] = phase_u.i;
		zero_seq_volt.io[1] = phase_v.i;
		zero_seq_volt.io[2] = phase_w.i;
		zero_seq_volt.redundant[0] = phase_u.redundant;
		zero_seq_volt.redundant[1] = phase_v.redundant;
		zero_seq_volt.redundant[2] = phase_w.redundant;
		zero_seq_volt.Inp_ref = ParaConvt.Cd*(udc2-udc1)/ParaConvt.Tp;
		zero_seq_volt.order_volt(&zero_seq_volt);
		zero_seq_volt.uz_calc(&zero_seq_volt);

		uou = 2+zero_seq_volt.uout[0];
		uov = 2+zero_seq_volt.uout[1];
		uow = 2+zero_seq_volt.uout[2];

	/*
		uoa = 2+uoa_ref+(-zero_seq_volt.umin0-zero_seq_volt.umax0)/2;
		uob = 2+uob_ref+(-zero_seq_volt.umin0-zero_seq_volt.umax0)/2;
		uoc = 2+uoc_ref+(-zero_seq_volt.umin0-zero_seq_volt.umax0)/2;

		uou = 2+uou_ref;
		uov = 2+uov_ref;
		uow = 2+uow_ref;
	*/
	//===============================================================//
	//根据三相参考电压计算层级、占空比，发出PWM信号	
		phase_u.usin = uou;
		phase_v.usin = uov;
		phase_w.usin = uow;
		
		phase_u.level_calc(&phase_u);
		phase_v.level_calc(&phase_v);
		phase_w.level_calc(&phase_w);

		phase_u.PWM_mode_calc(&phase_u);
		phase_v.PWM_mode_calc(&phase_v);
		phase_w.PWM_mode_calc(&phase_w);

		*PWMU_mode = phase_u.PWM_mode;
		*PWMV_mode = phase_v.PWM_mode;
		*PWMW_mode = phase_w.PWM_mode;
		
		pwm1.Tu = 1 - phase_u.duty;
		pwm1.Tv = 1 - phase_v.duty;
		pwm1.Tw = 1 - phase_w.duty;
		pwm1.Update(&pwm1);
	}
  }
  else
  {
	*PWMU_mode = 0x0000;	
	*PWMV_mode = 0x0000;	
	*PWMW_mode = 0x0000;
  }
}
//=====================================================================================================================================//
//scope读取参数
	INC_CLOCK();
//500
	scope_wave[0]=(int)(udc1*10);
	scope_wave[1]=(int)(udc2*10);
	scope_wave[2]=(int)(ufc_ref*10);
	scope_wave[3]=(int)((udc1-udc2)*1000);
//501
	scope_wave[4]=(int)(ufa*10);
	scope_wave[5]=(int)(ufb*10);
	scope_wave[6]=(int)(ufc*10);
//502
	scope_wave[7]=(int)(VcGrid.isa*1000);
	scope_wave[8]=(int)(VcGrid.isb*1000);
	scope_wave[9]=(int)(VcGrid.isc*1000);
//503
	scope_wave[10]=(int)(VcGrid.usa*10);
	scope_wave[11]=(int)(VcGrid.usb*10);
	scope_wave[12]=(int)(VcGrid.usc*10);
	scope_wave[13]=(int)(VcGrid.theta*100);
//504
	scope_wave[14]=(int)(uoa*1000);
	scope_wave[15]=(int)(uob*1000);
	scope_wave[16]=(int)(uoc*1000);
//505
	scope_wave[17]=(int)(VcGrid.id*1000);
	scope_wave[18]=(int)(VcGrid.iq*1000);
	scope_wave[19]=(int)(VcGrid.ud*1000);
	scope_wave[20]=(int)(VcGrid.uq*1000);
//506	
	scope_wave[21] = (int)(VcGrid.id_ref*1000);
	scope_wave[22] = (int)(VcGrid.PI_id.out*100);
	scope_wave[23] = (int)(VcGrid.PI_iq.out*100);
//507
	scope_wave[31]=(int)(ufu*10);
	scope_wave[32]=(int)(ufv*10);
	scope_wave[33]=(int)(ufw*10);
//508
	scope_wave[34]=(int)(VcMotor.Isa_SI*1000);
	scope_wave[35]=(int)(VcMotor.Isb_SI*1000);
	scope_wave[36]=(int)(VcMotor.Isc_SI*1000);
//509
	scope_wave[37]=(int)(uou_ref*1000);
	scope_wave[38]=(int)(uov_ref*1000);
	scope_wave[39]=(int)(uow_ref*1000);
//510
	scope_wave[40]=(int)(pi_isq.pi_ref*1000);
	scope_wave[41]=(int)(pi_isq.pi_fdb*1000);
	scope_wave[42]=(int)(pi_isd.pi_ref*1000);
	scope_wave[43]=(int)(pi_isd.pi_fdb*1000);
//511
	scope_wave[44]=(int)(VcMotor.Te_ref*1000);
	scope_wave[45]=(int)(VcMotor.wr_meca_SI/DPI*60);
	scope_wave[46]=(int)(fe1.amp_r_fe*1000);
	scope_wave[47]=(int)(fe1.wr_elec_es*1500);
	scope_wave[48]=(int)(VcMotor.flux_theta*500);


	
	if ((Add_Val>=100)&&(Add_Val<200))
		scope_index0 = (Add_Val-100)%50;
	else if ((Add_Val>=200)&&(Add_Val<300))
		scope_index1 = (Add_Val-200)%50;
	else if ((Add_Val>=300)&&(Add_Val<400))
		scope_index2 = (Add_Val-300)%50;
	else if ((Add_Val>=400)&&(Add_Val<500))
		scope_index3 = (Add_Val-400)%50;

	else if ((Add_Val>=1000)&&(Add_Val<=1500))
		speed_ref = (Add_Val-1000.0)/10.0;
	else if ((Add_Val>=-1500)&&(Add_Val<=-1000))
		speed_ref = (Add_Val+1000.0)/10.0;
//		udc_ref= (Add_Val-1000)*10;

	else if ((Add_Val>=4000)&&(Add_Val<=4100))
		iq_ref_grid = (Add_Val-4000.0)/10;
	else if ((Add_Val>=-4100)&&(Add_Val<=-4000))
		iq_ref_grid = (Add_Val+4000.0)/10;




//	else if ((Add_Val>=1100)&&(Add_Val<1200))
//		VcGrid.PI_iq.kp = (Add_Val-1100.0)/10;
//	else if ((Add_Val>=1200)&&(Add_Val<1300))
//		VcGrid.PI_iq.ki = (Add_Val-1200.0)/1000;

	else if ((Add_Val>=2100)&&(Add_Val<2200))
		pi_isd.Kp = (Add_Val-2100.0)/10;
	else if ((Add_Val>=2200)&&(Add_Val<2300))
		pi_isd.Ki = (Add_Val-2200.0)/10;

	else if ((Add_Val>=3100)&&(Add_Val<3200))
		pi_isq.Kp = (Add_Val-3100.0)/100;
	else if ((Add_Val>=3200)&&(Add_Val<3300))
		pi_isq.Ki = (Add_Val-3200.0)/100;


	
	else if (Add_Val==500)
	{
		scope_index0=0;
		scope_index1=1;
		scope_index2=2;
		scope_index3=3;
	}
	else if (Add_Val==501)
	{
		scope_index0=3;
		scope_index1=4;
		scope_index2=5;
		scope_index3=6;
	}
	else if (Add_Val==502)
	{
		scope_index0=3;
		scope_index1=7;
		scope_index2=8;
		scope_index3=9;
	}
	else if (Add_Val==503)
	{
		scope_index0=10;
		scope_index1=11;
		scope_index2=12;
		scope_index3=13;
	}
	else if (Add_Val==504)
	{
		scope_index0=13;
		scope_index1=14;
		scope_index2=15;
		scope_index3=16;
	}
	else if (Add_Val==505)
	{
		scope_index0=17;
		scope_index1=18;
		scope_index2=19;
		scope_index3=20;
	}
	else if (Add_Val==506)
	{
		scope_index0=3;
		scope_index1=21;
		scope_index2=22;
		scope_index3=23;
	}
	else if (Add_Val==507)
	{
		scope_index0=3;
		scope_index1=31;
		scope_index2=32;
		scope_index3=33;
	}
	else if (Add_Val==508)
	{
		scope_index0=3;
		scope_index1=34;
		scope_index2=35;
		scope_index3=36;
	}
	else if (Add_Val==509)
	{
		scope_index0=3;
		scope_index1=37;
		scope_index2=38;
		scope_index3=39;
	}
	else if (Add_Val==510)
	{
		scope_index0=40;
		scope_index1=41;
		scope_index2=42;
		scope_index3=43;
	}
	else if (Add_Val==511)
	{
		scope_index0=44;
		scope_index1=45;
		scope_index2=46;
		scope_index3=47;
	}
	else if (Add_Val==512)
	{
		scope_index0=0;
		scope_index1=1;
		scope_index2=2;
		scope_index3=3;
	}
	else if (Add_Val==513)
	{
		scope_index0=0;
		scope_index1=1;
		scope_index2=2;
		scope_index3=3;
	}
/*	
	else if (Add_Val==1010)
	{
		speed_ref = 10;
//		m=0.2;
	}
	else if (Add_Val==1020)
	{
		speed_ref = 20;
//		m=0.4;
	}
	else if (Add_Val==1030)
	{
		speed_ref = 30;
//		m=0.6;
	}
	else if (Add_Val==1040)
	{
 		speed_ref = 40;
//		m=0.8;
	}
	else if (Add_Val==1050)
	{
		speed_ref = 50;
//		m=1;
	}*/
	else
	{
		scope_index0=44;
		scope_index1=40;
		scope_index2=41;
		scope_index3=45;
	}

		Val_Channel[0] = scope_wave[scope_index0];
		Val_Channel[1] = scope_wave[scope_index1];
		Val_Channel[2] = scope_wave[scope_index2];
		Val_Channel[3] = scope_wave[scope_index3];

	SAVETOBUFFER();
//===============================================//
	EPwm1Regs.ETCLR.bit.SOCA = 1;
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;		// Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all |= PIEACK_GROUP1;
}
//=======================================================================================================//
interrupt void cpu_timer0_isr(void)
{
   
   CpuTimer0.InterruptCount++;

//-----------------------------------------//
/*
	LED_count++;
		alarm_VVVF();*/
//-----------------------------------------------------------//
   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
   CpuTimer0Regs.TCR.bit.TIF=1;
   CpuTimer0Regs.TCR.bit.TRB=1;
}

/*interrupt void cpu_timer1_isr(void)
{
   CpuTimer1.InterruptCount++;
   // The CPU acknowledges the interrupt.
   EDIS;
}

interrupt void cpu_timer2_isr(void)
{  EALLOW;
   CpuTimer2.InterruptCount++;
   // The CPU acknowledges the interrupt.
   EDIS;
}*/

interrupt void TZ_int(void)
{
PieCtrlRegs.PIEACK.all |= PIEACK_GROUP2;
}
//====================================================================================================//
void scib_echoback_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.all =0x0003;
	ScibRegs.SCICTL2.bit.TXINTENA =1;//enable the interrupt
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;

	#if (CPU_FRQ_150MHZ)
	    //  SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	   //   SciaRegs.SCILBAUD    =0x00E7;
		ScibRegs.SCIHBAUD    =0x0000;  // 115200 baud @LSPCLK = 37.5MHz.
	    ScibRegs.SCILBAUD    =0x0028;
	#endif
	#if (CPU_FRQ_100MHZ)
      ScibRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      ScibRegs.SCILBAUD    =0x0044;
	#endif

	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI

// Initalize the SCI FIFO
void scib_fifo_init()
{
/*    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x204f;
    SciaRegs.SCIFFCT.all=0x0;
*/
	ScibRegs.SCIFFTX.all=0xA040;
    ScibRegs.SCIFFRX.all=0x004f;
    ScibRegs.SCIFFCT.all=0x0;

}

//==============================================================================================

//===========================================================================================//


