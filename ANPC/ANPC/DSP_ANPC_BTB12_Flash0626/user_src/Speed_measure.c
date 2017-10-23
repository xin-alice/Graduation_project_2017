#include "Speed_measure.h"
#include "personal_set.h"
interrupt void qep1_int(void);
eQEP qep1 = eQEP_DEFAULTS;///////////////


void eQEP_init(eQEP *p)
{

	p->Fs_cpu_M = 150;//  cpu frequency is 150M Hz
	p->Ts = 0.000125;//para.Ts/2.0;//16k//0.000125s
	p->Ts_speed = p->Ts*10.0;//qep1.Ts;//0.000125*10
	p->encoder_linear = 1000;//1000;//1000 P/R
	p->encoder_linear_inv = 1.0/(p->encoder_linear);
	p->qep_spd_cnt = 10;

//-----------------------------------------//
	EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
	EQep1Regs.QDECCTL.bit.XCR = 1;// count the rising edge
	EQep1Regs.QDECCTL.bit.SWAP = 0;// quadrature-clock inputs are not swapped  1: quadrature-clock inputs are swapted
	EQep1Regs.QDECCTL.bit.QIP = 0;// 0: no effect; 1: negates QEPI input
//-----------------------------------------//
	EQep1Regs.QEPCTL.bit.PCRM = 0;// position counter reset on an index event （0） or maximum position(1) or first index (2) or unit timer evernt(3)

	EQep1Regs.QEPCTL.bit.IEI = 0;// do nothing: Index event initialization of position counter
	EQep1Regs.QEPCTL.bit.IEL = 0;//1: latches position counter on rising edge of the index signal
//	EQep1Regs.QPOSILAT = 0;
	EQep1Regs.QEPCTL.bit.QPEN = 1;// enable the couter
	EQep1Regs.QEPCTL.bit.UTE = 1;//enable eQEP unit timer
	EQep1Regs.QEPCTL.bit.WDE = 0;// disable the eQEP watch dog timer

//-----------------------------------------//	
	EQep1Regs.QPOSMAX = p->encoder_linear*4;// 10000;//para.encoder_linear*4;//
	EQep1Regs.QPOSINIT = 0;//initial value of the counter 
//-----------------------------------------//
	EQep1Regs.QPOSLAT = 0;//The position-counter value is latched into this register on unit time out event.
//	This register acts as time base for unit time event generation. When this timer value matches
//with unit time period value, unit time event is generated.

	EQep1Regs.QUTMR = 0;
	EQep1Regs.QUPRD = p->Ts*p->Fs_cpu_M*1000000;//15000;// 100us// p->Ts*p->Fs_cpu_M*1000000/2;/para.qep_UTO_period;//
//-----------------------------------------//
	EQep1Regs.QEINT.bit.UTO = 1;// enable the unit time out interrupt 
	EQep1Regs.QCLR.bit.UTO = 1;// clear the Unit timer int flag	 
	EQep1Regs.QEINT.bit.IEL = 1;//Index event latch interrupt enable
	EQep1Regs.QCLR.bit.IEL = 1;//Clear index event latch interrupt flag
//===================================================================================//
// for the capture
	EQep1Regs.QCAPCTL.bit.CEN = 1;// enable capture
	EQep1Regs.QCAPCTL.bit.CCPS = 7 ;// capclk = sysclkout/128  = 1.171875M
	EQep1Regs.QCAPCTL.bit.UPPS = 0; // unit position event prescaler  by 1
	EQep1Regs.QEPCTL.bit.QCLM = 1;// latch the position counter capture timer and capture period on unit time out
//===================================================================================//


	EALLOW;
 	 PieVectTable.EQEP1_INT = &qep1_int;
   EDIS;
   IER |= M_INT5; // for the eQEP int
	PieCtrlRegs.PIEIER5.bit.INTx1 = 1;// for the eQEP1 int


}
//==================================================================================//



#pragma CODE_SECTION(qep1_int, "ramfuncs")
void qep1_int(void)
{
// INT5.1

//----------------------------------------------------------------------------------------------------------//
	if (EQep1Regs.QFLG.bit.IEL ==1) //index event
	{
		EQep1Regs.QCLR.bit.IEL = 1;//clear the interupt flag
		qep1.index_flt_cnt = 0;
		EQep1Regs.QPOSCNT = 0;
		//---------------//
		if 	(EQep1Regs.QEPSTS.bit.QDF ==1) //Clockwise rotation (or forward movement)
		{
//			qep1.cycle_cnt ++;
		}
		else  //Counter-clockwise rotation (or reverse movement)
		{
//			qep1.cycle_cnt--;
		}
		if (EQep1Regs.QEPSTS.bit.FIMF ==1)//First index marker flag,Set by first occurrence of index pulse
		{
//			qep1.cycle_cnt = 0;
			qep1.first_index = 1;// the first index signal of the encoder is passed.
			EQep1Regs.QEPSTS.bit.FIMF =1;//Sticky bit, cleared by writing 1
		}
		else
			qep1.first_index = 2;

		   
	}

//----------------------------------------------------------------------------------------------------------//


	if (EQep1Regs.QFLG.bit.UTO ==1) // 125/2 us  Ts/2
	{
		EQep1Regs.QCLR.bit.UTO = 1; //clear the unit timer flag
//		qep1.posi_cnt = EQep1Regs.QPOSCNT;
		qep1.posi_cnt = EQep1Regs.QPOSLAT;						//转子当前位置脉冲计数锁存值
//		qep1.posi_meca = qep1.posi_cnt*0.0001*DPI;
		qep1.posi_meca = qep1.posi_cnt*0.25*qep1.encoder_linear_inv*6.2831853;	//计算机械位置角度
		qep1.posi_cnt_old = qep1.posi_cnt;
//--------------------------------------------------------------------//
		qep1.speed_meas_cnt++;									
		if (qep1.speed_meas_cnt >qep1.qep_spd_cnt)//10				//每隔10个中断周期计算一次转速
		{
			qep1.delta_posi = qep1.posi_meca - qep1.posi_meca_old;	//转子位置差
			if (qep1.delta_posi > 3.1415926)						//位置角度变化不可能大于pi，除非反转
			{
				qep1.delta_posi = qep1.delta_posi - 6.2831853;	
			}
			if (qep1.delta_posi < -3.1415926)
			{
				qep1.delta_posi = qep1.delta_posi + 6.2831853;
			}
			qep1.w_meca = qep1.delta_posi/qep1.Ts_speed;		//转子机械速度计算
		//--------------------------------------// 

		// calculation for the first index signal
			if (qep1.first_index == 1)
			{
				qep1.w_meca = qep1.w_mec_old;
			}
			else
				qep1.w_meca = qep1.delta_posi/qep1.Ts_speed;
		//--------------------------------------// 
			qep1.w_mec_old = qep1.w_meca;				
			qep1.posi_meca_old = qep1.posi_meca;
			qep1.speed_meas_cnt = 1;
		}
//			qep1.w_elec_pu = qep1.w_elec/para.BASE_w;
//-------------------------------------------------------------------------//
// cap model
		
		if ((EQep1Regs.QEPSTS.bit.CDEF ==0)&&(EQep1Regs.QEPSTS.bit.COEF ==0))
		{
			qep1.cap_cnt = EQep1Regs.QCPRD;
	//		qep1.cap_deltaT = _IQmpy();
			qep1.cap_speed = 3.515625/qep1.cap_cnt;//??????????
			if(qep1.dir_QEP ==0 )//reverse
              qep1.cap_speed = -qep1.cap_speed;
		}
		else if(EQep1Regs.QEPSTS.bit.COEF ==1)
		{
			qep1.cap_speed = 0; // capture timer over flowed
			EQep1Regs.QEPSTS.bit.COEF = 1;
		}
		else if(EQep1Regs.QEPSTS.bit.CDEF ==1)// running direction changed between the capture
		{
			qep1.cap_speed = 0; //
			EQep1Regs.QEPSTS.bit.CDEF =1;
		}
		
	}
//--------------------------------------------------------------------------------//


		EQep1Regs.QCLR.bit.INT = 1;
		PieCtrlRegs.PIEACK.all |= PIEACK_GROUP5;
}
//====================================================================================================//
