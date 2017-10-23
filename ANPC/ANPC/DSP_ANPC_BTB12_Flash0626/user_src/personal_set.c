#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "personal_set.h"
//#include "vector_control.h"
//==================================================================================//
extern int LED_count;
extern int LED_num;
extern int LED_count2;
extern int Ext_Ctrl;
//==================================================================================//

void Personal_set(para_converter *p)
{
//===================================================================================//
//-----------------------------------------------------------//
//initial for short and brake
   EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;// as IO    short
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;//brake
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1; //as output
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;
//	GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0; // enable pull-up
//	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;
	EDIS;
//=====================================================================================//
//	InitEQep1Gpio();
//-----------------------------------------//
	EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
	EQep1Regs.QDECCTL.bit.XCR = 0;// count the rising edge:1; Count the rising/falling edge:0
	EQep1Regs.QDECCTL.bit.SWAP = 0;// quadrature-clock inputs are not swapped  1: quadrature-clock inputs are swapted
	EQep1Regs.QDECCTL.bit.QIP = 0;// 0: no effect; 1: negates QEPI input
//-----------------------------------------//
	EQep1Regs.QEPCTL.bit.PCRM = 0;// position counter reset on an index event £¨0£© or maximum position(1) or first index (2) or unit timer evernt(3)

	EQep1Regs.QEPCTL.bit.IEI = 0;// do nothing: Index event initialization of position counter
	EQep1Regs.QEPCTL.bit.IEL = 0;//1: latches position counter on rising edge of the index signal
//	EQep1Regs.QPOSILAT = 0;
	EQep1Regs.QEPCTL.bit.QPEN = 1;// enable the couter
	EQep1Regs.QEPCTL.bit.UTE = 1;//enable eQEP unit timer
	EQep1Regs.QEPCTL.bit.WDE = 0;// disable the eQEP watch dog timer

//-----------------------------------------//	
	EQep1Regs.QPOSMAX = p->encoder_linear*4;//10000;
	EQep1Regs.QPOSINIT = 0;//initial value of the counter 
//-----------------------------------------//
	EQep1Regs.QPOSLAT = 0;//The position-counter value is latched into this register on unit time out event.
//	This register acts as time base for unit time event generation. When this timer value matches
//with unit time period value, unit time event is generated.

	EQep1Regs.QUTMR = 0;	//This register acts as time base for unit time event generation. When this timer value matches
							//with unit time period value, unit time event is generated.
	EQep1Regs.QUPRD = 150000000*0.00025;//15000;// 100us// p->Ts*p->Fs_cpu_M*1000000/2;
							//This register contains the period count for unit timer to generate periodic unit time events to latch
							//the eQEP position information at periodic interval and optionally to generate interrupt.
//-----------------------------------------//
	EQep1Regs.QEINT.bit.UTO = 1;// enable the unit time out interrupt 
	EQep1Regs.QCLR.bit.UTO = 1;// clear the Unit timer int flag	 
	EQep1Regs.QEINT.bit.IEL = 1;//Index event latch interrupt enable
	EQep1Regs.QCLR.bit.IEL = 1;//Clear index event latch interrupt flag
//=================================================================//
// for the capture
	EQep1Regs.QCAPCTL.bit.CEN = 1;// enable capture
	EQep1Regs.QCAPCTL.bit.CCPS = 7 ;// capclk = sysclkout/128  = 1.171875M
	EQep1Regs.QCAPCTL.bit.UPPS = 0; // unit position event prescaler  by 1
	EQep1Regs.QEPCTL.bit.QCLM = 1;// latch the position counter capture timer and capture period on unit time out
	EQep1Regs.QEPCTL.bit.SWI = 1;//Software initialization of position counter
//=========================================================================================================//
	EPwm1Regs.TBPRD = 150000000/p->fs/2;// 2kHz   
	EPwm1Regs.TBPHS.all = 0;     // the counter value when there is a synchronize signal
	EPwm1Regs.TBCTL.bit.CLKDIV = 0;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;// the timer frequency of the PWM module is 150M Hz
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;// select the synchronize signal
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;// set the action when comparision A event
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;// set the action when comparision A event
	EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

//	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//	EPwm1Regs.DBFED = PWM_DBFED;// dead band falling edge 2us
//	EPwm1Regs.DBRED = PWM_DBRED;//dead band rising edge 2us

//--------------------------------------------------------------------------------------------//
	EPwm2Regs.TBPRD = 150000000/p->fs/2;
	EPwm2Regs.TBPHS.all = 0;
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;// 0x0 from the former pwm unit
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
	EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;
//	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//	EPwm2Regs.DBFED = PWM_DBFED;
//	EPwm2Regs.DBRED = PWM_DBRED;
//--------------------------------------------------------------------------------------------//
	EPwm3Regs.TBPRD = 150000000/p->fs/2;
	EPwm3Regs.TBPHS.all = 0;
	EPwm3Regs.TBCTL.bit.CLKDIV = 0;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
	EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;
//	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//	EPwm3Regs.DBFED = PWM_DBFED;
//	EPwm3Regs.DBRED = PWM_DBRED;
//--------------------------------------------------------------//
	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;

//==============================================================//
/*
 InitTzGpio();
   EALLOW;	
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
   EDIS;

//	EPwm1Regs.TZSEL.bit.CBC1 = TZ_ENABLE;
	EPwm1Regs.TZSEL.bit.OSHT1 = TZ_ENABLE; 
//	EPwm1Regs.TZSEL.bit.CBC2 = TZ_ENABLE;
	EPwm1Regs.TZSEL.bit.OSHT2 = TZ_ENABLE;
	EPwm1Regs.TZCTL.bit.TZA = TZ_HIZ;//TZ_FORCE_LO;
	EPwm1Regs.TZCTL.bit.TZB = TZ_HIZ;//TZ_FORCE_LO;
	EPwm1Regs.TZEINT.bit.CBC = 1;
	EPwm1Regs.TZEINT.bit.OST = 1;
	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm1Regs.TZCLR.bit.CBC = 1;
	EPwm1Regs.TZCLR.bit.INT = 1;

*/
}

//==================================================================================//
#pragma CODE_SECTION(PWM_Update_calc, "ramfuncs")
void PWM_Update_calc(PWM_Update *v)
{
	EPwm1Regs.CMPA.half.CMPA = (int)(v->Ta*v->PWM_period);
	EPwm2Regs.CMPA.half.CMPA = (int)(v->Tb*v->PWM_period);
	EPwm3Regs.CMPA.half.CMPA = (int)(v->Tc*v->PWM_period);

	EPwm1Regs.CMPB = (int)(v->Tu*v->PWM_period);
	EPwm2Regs.CMPB = (int)(v->Tv*v->PWM_period);
	EPwm3Regs.CMPB = (int)(v->Tw*v->PWM_period);
}
//==================================================================================//
#pragma CODE_SECTION(alarm_Normal, "ramfuncs")
void alarm_Normal(void)//normal
{
	if(LED_count >400)
	{
		LED_count = 0;
		LED_count2 ++;
		LED_num = LED_num<<1;
		
		if(LED_count2 > 8)
		{
			LED_num = 1;
			LED_count2 = 0;
			
		}
		*ext_out = (LED_num & 0x00FF) | (Ext_Ctrl<<8);	
	}
}


#pragma CODE_SECTION(alarm_Hardware, "ramfuncs")
void alarm_Hardware(void)  //
{
	if(LED_count >1600)
	{
		LED_count = 0;
		if(LED_num == 0)
		{
			LED_num = 1;
			*ext_out = 0x00FF | (Ext_Ctrl<<8);  // 1111 0000
			

		}
		else
		{
			LED_num = 0;
			*ext_out = 0x0000 | (Ext_Ctrl<<8);  // 0000 0000

		}
	}	
}	

#pragma CODE_SECTION(alarm_Software, "ramfuncs")
void alarm_Software(void)
{
	if(LED_count >1600)
	{
		LED_count = 0;
		if(LED_num == 0)
		{
			LED_num = 1;
			*ext_out = 0x000f | (Ext_Ctrl<<8);  // 0000 1111 
		}
		else
		{
			LED_num = 0;
			*ext_out = 0x00f0 | (Ext_Ctrl<<8);  // 1111 0000
		}
	}	
}

#pragma CODE_SECTION(alarm_PWMdisable, "ramfuncs")
void alarm_PWMdisable(void)
{
	if(LED_count >1600)
	{
		LED_count = 0;
		if(LED_num == 0)
		{
			LED_num = 1;
			*ext_out = 0x00AA | (Ext_Ctrl<<8);  // 1010 1010
		}
		else
		{
			LED_num = 0;
			*ext_out = 0x0055 | (Ext_Ctrl<<8);  // 0101  0101
		}
	}	
}

#pragma CODE_SECTION(alarm_brake, "ramfuncs")
void alarm_brake(void)
{
	if(LED_count >1600)
	{
		LED_count = 0;
		if(LED_num == 0)
		{
			LED_num = 1;
			*ext_out = 0x0033 | (Ext_Ctrl<<8);// 0011 0011
		}
		else
		{
			LED_num = 0;
			*ext_out = 0x00CC | (Ext_Ctrl<<8);  // 1100 1100
		}
	}	

}

//==================================================================================//

//================================================================================//


