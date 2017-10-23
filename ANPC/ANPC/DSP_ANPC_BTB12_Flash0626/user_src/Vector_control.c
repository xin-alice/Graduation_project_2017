#include "Vector_control.h"
#include "common_struct.h"
#include <math.h>

//======================================================================================================================================//

//#pragma CODE_SECTION(Vector_ctrl_reset, "ramfuncs")
//#pragma CODE_SECTION(flux_estimate, "ramfuncs")
//#pragma CODE_SECTION(Para_derive, "ramfuncs")

//#pragma CODE_SECTION(pi_speed_calc, "ramfuncs")
//#pragma CODE_SECTION(pi_fun_calc, "ramfuncs")
//#pragma CODE_SECTION(park_calc, "ramfuncs")
//#pragma CODE_SECTION(ipark_calc, "ramfuncs")
//#pragma CODE_SECTION(clarke_calc, "ramfuncs")
//#pragma CODE_SECTION(iclarke_calc, "ramfuncs")
//======================================================================================================================================//

//------------------------------------------------//

	Parameter_set para = Parameter_set_DEFAULTS;
	Parameter_used para2;
	 
//------------------------------------------------//
	
	LPF	lpf_isd = LPF_DEFAULTS;
	LPF	lpf_isq = LPF_DEFAULTS;

	LPF	lpf_w = LPF_DEFAULTS;
	LPF	lpf_w_ref = LPF_DEFAULTS;
//=======================================================================================//

	PI_SPEED_CONST pi_speed_const = PI_SPEED_CONST_DEFAULTS;
	PI_CURRENT_CONST pi_current_const = PI_CURRENT_CONST_DEFAULTS;
	
	PI_SPEED	pi_speed = PI_SPEED_DEFAULTS;
	PI_SPEED	pi_posi = PI_SPEED_DEFAULTS;
	PI_fun		pi_isd = PI_DEFAULTS;
	PI_fun		pi_isq = PI_DEFAULTS;
	PI_flux		pi_flux_w = PI_flux_DEFAULTS;// PI for the adaptive flux weakening
	//------------------------------------------------//
	// variables for the flux estimation
	FE fe1 = FE_DEFAULTS;

	LPF  lpf_al_er = LPF_DEFAULTS;
	LPF  lpf_be_er = LPF_DEFAULTS;
	LPF  lpfpf_al_er = LPF_DEFAULTS;
	LPF  lpfpf_be_er = LPF_DEFAULTS;
	LPF  lpfpfpf_al_er = LPF_DEFAULTS;
	LPF  lpfpfpf_be_er = LPF_DEFAULTS;
	LPFI lpfi_al = LPFI_DEFAULTS;
	LPFI lpfi_be = LPFI_DEFAULTS;
	LPF  lpf_al_fe_i = LPF_DEFAULTS;
	LPF  lpf_be_fe_i = LPF_DEFAULTS;
//======================================================================================================================================//
void Vector_ctrl_init(Vector_ctrl *v)
{
//--------------------------------------------------------------------------------------------//
	para.Fs_cpu_M = 150;//150MHz
//---------------------------------------------------------//
	para.Ts = 0.0005;// sample time 500us  2K Hz //Ts;

	para.P_N = 2200.0;//750;//
	para.UL_N = 380.0;//380
	para.I_N = 4.9;//1.65;//5.0;//
	para.rpm_N = 1430.0;
	para.F_N = 50;// rated frequency
	para.p = 2;
//------------------------------------------//
// para to be tested
	para.Rs = 2.74;
	para.Rr = 2.54;
	para.Ls = 0.3128;
	para.Lr = 0.3128;
	para.Lm = 0.299;
	para.friction = 0.000128;
	para.Inertia = 0.00072;

//------------------------------------------//
	para.H = 5;
	para.Wc_wr = 10.0*2.0*PI;
	para.Wc_cur = 500.0*2.0*PI;
	para.Wc_offset = 0.6;

//--------------------------------------------------------------------------------------------//
	v->p_speed_coe = 20.0;//1.0;	//3	
	v->i_speed_coe = 0.01;//0.01;
	v->p_isd_coe = 0.5;		
	v->i_isd_coe = 0.1;		
	v->p_isq_coe = 0.1;		
	v->i_isq_coe = 0.1;	
//------------------------------------------//
	v->flux_level = 1.0;	
	v->k_comp = 0.0;
	v->control_method = 1;// with speed sensor
	v->start = 1;
	v->rmp_delay_cntl = 0;
	v->rmp_dly_max = 100;

//	Ts= PWM_Period*2/150000000;
// parameters for PMSM 750W

	para.derive(&para);
//--------------------------------------------------------------------------------------------//
// parameter used in the control module p.u.
	para2.Ls = para.ls;
	para2.Lr = para.lr;
	para2.Rs = para.rs;
	para2.Ld = para.ld;//not used
	para2.Lq = para.lq;//not used
	para2.Lm = para.lm;
	para2.Rr = para.rr;
	para2.LrbyLm = para2.Lr/para2.Lm;
	para2.LmbyLr = para2.Lm/para2.Lr;
	para2.Tr = para2.Lr/para2.Rr;
	para2.Tr_invt = 1.0/para2.Tr;
	para2.Lsig = para2.Ls*(1.0-para2.Lm*para2.Lm/(para2.Ls*para2.Lr));
	para2.p = para.p;
	para2.flux_d = para.flux_d_pu;//not used
	para2.flux_r = para.flux_r_pu;//not used
	para2.Inertia = para.inertia_pu;
	para2.friction = para.friction_pu;
	para2.Wc_cur = para.wc_cur_pu;
	para2.Wc_wr = para.wc_wr_pu;
	para2.Wc_offset = para.wc_offset_pu;
	para2.Imax = para.Imax_pu;
	para2.Umax = para.Umax_pu;
	para2.Idq_max = para.Idq_max_pu;
	para2.Udq_max = para.Udq_max_pu;
	para2.Torque_max = para.Torque_max_pu;
	para2.Freq_max = para.freq_max_pu;
	para2.flux_max = para.flux_max_pu;
//	para2.flux_ref = para.flux_ref_pu;
	para2.flux_ref = para.flux_ref_dq_pu;
	para2.Ts = para.ts;
//------------------------------------------------------------//

//VVVF model
	para2.VVVF_step_freq = 0.1/para.BASE_FREQ;
	para2.VVVF_base_volt = 1.0*1.0;//
	
//------------------------------------------------------------//
//===================================================================================//
	fe1.Rs = para2.Rs;
	fe1.Lm = para2.Lm;
	fe1.LrbyLm = para2.LrbyLm;
	fe1.Lsig = para2.Lsig;
	fe1.Tr_invt = para2.Tr_invt;
	fe1.tc = para2.Ts;
	fe1.wc = para2.Wc_wr;
	fe1.flux_ref = para2.flux_ref;
	fe1.slip_coe = 1.0;
//===================================================================================//

//--------------------------------------------//
// for the current model in flux estimation
	lpfi_al.wc = para2.Tr_invt;
	lpfi_be.wc = para2.Tr_invt;
	lpfi_al.Tc = para2.Ts;
	lpfi_be.Tc = para2.Ts;
/* Initialize the LPF module */	
	lpf_al_fe_i.wc = para2.Tr_invt;
	lpf_be_fe_i.wc = para2.Tr_invt;
	lpf_al_fe_i.Tc = para2.Ts;
	lpf_be_fe_i.Tc = para2.Ts;
//--------------------------------------------//
	lpf_al_er.wc = para2.Wc_offset;
	lpf_be_er.wc = para2.Wc_offset;
	lpf_al_er.Tc = para2.Ts;
	lpf_be_er.Tc = para2.Ts;
	lpfpf_al_er.wc = para2.Wc_offset;
	lpfpf_be_er.wc = para2.Wc_offset;
	lpfpf_al_er.Tc = para2.Ts;
	lpfpf_be_er.Tc = para2.Ts;
	lpfpfpf_al_er.wc = para2.Wc_offset;
	lpfpfpf_be_er.wc = para2.Wc_offset;
	lpfpfpf_al_er.Tc = para2.Ts;
	lpfpfpf_be_er.Tc = para2.Ts;

//--------------------------------------------------------------------------------------------//	
	lpf_w.wc = para2.Wc_wr; //para.wc_wr_pu;
	lpf_w.Tc = para2.Ts;// para.ts;
	lpf_w_ref.wc  = para2.Wc_wr;
	lpf_w_ref.Tc = para2.Ts; 
	lpf_isd.wc = para2.Wc_wr;// para.wc_wr_pu;
	lpf_isd.Tc = para2.Ts;// para.ts;
	lpf_isq.wc = para2.Wc_wr;//   para.wc_wr_pu;
	lpf_isq.Tc = para2.Ts;//  para.ts;
	//===============================================================================================//

	pi_current_const.wc_current = para2.Wc_cur;//  para.wc_cur_pu;
	pi_current_const.ls = para2.Ls;//  para.ls;
	pi_current_const.rs = para2.Rs;//  para.rs;
	pi_current_const.calc(&pi_current_const);
//-------------------------------------------------------//
//	pi_speed.Ki = pi_speed_const.Ki*v->i_speed_coe;
//	pi_speed.Kp = pi_speed_const.Kp*v->p_speed_coe;

	pi_speed.Ki = v->i_speed_coe;
	pi_speed.Kp = v->p_speed_coe;
	pi_speed.pi_out_max = para2.Torque_max;//para2.Idq_max;//  para.Idq_max_pu;
	pi_speed.Tc = para2.Ts*10.0;// para.ts*10.0;
//------------------------------------------------------------//
	pi_posi.Ki = v->i_posi_coe;
	pi_posi.Kp = v->p_posi_coe;
	pi_posi.pi_out_max = para2.Freq_max*1.0;
	pi_posi.Tc = para2.Ts*10.0;
//------------------------------------------------------------//
	pi_isd.Ki = v->i_isd_coe;
	pi_isd.Kp = v->p_isd_coe;
	pi_isd.pi_var_max = para2.Udq_max;//  para.Udq_max_pu;
	pi_isd.pi_out_max = para2.Udq_max;// para.Udq_max_pu;
	pi_isd.Tc = para2.Ts;// para.ts;

	pi_isq.Ki = v->i_isq_coe;
	pi_isq.Kp = v->p_isq_coe;
	pi_isq.pi_var_max = para2.Udq_max;// para.Udq_max_pu;
	pi_isq.pi_out_max = para2.Udq_max;// para.Udq_max_pu;
	pi_isq.Tc = para2.Ts; //para.ts;
//------------------------------------------------------------//
	pi_flux_w.Kp = v->p_flux_coe;
	pi_flux_w.Ki = v->i_flux_coe;
	pi_flux_w.Tc = para2.Ts;
	pi_flux_w.pi_out_max = 0.1;//para2.Idq_max;
	pi_flux_w.pi_out_min = -para2.Idq_max;
//------------------------------------------------------------//

	
}
//===============================================================================================//
#pragma CODE_SECTION(Vector_ctrl_reset, "ramfuncs")
void Vector_ctrl_reset(Vector_ctrl *v)
{

	pi_speed.ui = 0.0;
	pi_isd.ui = 0.0;
	pi_isq.ui = 0.0;
/*
	v->Vsd_ref = 0.0;
	v->Vsq_ref = 0.0;
	v->flux_delay_cnt = 0;
	v->speed_pi_count = 0;
	v->start = 0;

	fe1.amp_r_fe = 0;
	fe1.er_al = 0;
	fe1.er_be = 0;
	fe1.flux_v_amp = 0;
	fe1.theta_r_fe = 0;
	fe1.wr_elec_es_flt = 0;
	fe1.w_slip_flt = 0;
	fe1.w_syn_flt = 0;
*/
	lpf_al_er.y = 0;
	lpf_be_er.y = 0;
	lpfpf_al_er.y = 0;
	lpfpf_be_er.y = 0;
	lpfpfpf_al_er.y = 0;
	lpfpfpf_be_er.y = 0;

	lpfi_al.y = 0;
	lpfi_be.y = 0;

	lpf_al_fe_i.y = 0;
	lpf_be_fe_i.y = 0;

	v->control_method = 0;// if 1 there is sensor, 0 without sensor,3 VVVF
	v->flux_delay_cnt = 0;// to build the rotor flux
	v->start = 0;

//input
	v->Isa_SI = 0;
	v->Isb_SI = 0;
	v->Isc_SI = 0;
	v->wr_meca_SI = 0;
	v->posi_meca = 0;           /* Speed reference given by PC scope */
	v->posi_elec = 0;
	v->posi_ab = 0;//转子的绝对位置
	v->speed_ref_Hz = 0;
	v->speed_ref = 0;           /* Speed reference  */
	v->speed_ref_flt = 0;				 /* Speed reference after filter (pu) */
	v->posi_ref = 0;
//-------------------------------------------------//
//output
	v->Ualfa_ref = 0;
	v->Ubeta_ref = 0;
	v->Ualfa_ref_pu = 0;
	v->Ubeta_ref_pu = 0;
	v->Ua_ref = 0;
	v->Ub_ref = 0;
	v->Uc_ref = 0;
//-------------------------------------------------//
	v->wr_elec = 0;
	v->wr_meca = 0;
//-------------------------------------------------//
	v->Isa = 0;
	v->Isb = 0;
	v->Isc = 0;
	v->speedback = 0;
	v->Isa_flt = 0;
	v->Isb_flt = 0;
	v->Isc_flt = 0;
//-------------------------------------------------//				
	v->speed_pi_count = 0;
	v->posi_pi_count = 0;
	v->p_posi_coe = 0;//0.1 = 0;
	v->i_posi_coe = 0;
	v->p_speed_coe = 0;//1.0 = 0;	//3	
	v->i_speed_coe = 0;//0.01 = 0;
	v->p_isd_coe = 0;		
	v->i_isd_coe = 0;		
	v->p_isq_coe = 0;		
	v->i_isq_coe = 0;
	v->p_flux_coe = 0;
	v->i_flux_coe = 0;
//--------------------------------------------------//		
	v->ISA = 0;
	v->ISB = 0;
	v->flux_theta = 0;
	v->cos_theta = 0;
	v->sin_theta = 0;
	v->Te_ref = 0;				/* reference electromagnetic torque  */
	v->isd_ref = 0;			/* reference stator d-axis current  */
	v->isq_ref = 0;			/* reference stator q-axis current  */
	v->Vsd_ref = 0;			/* reference stator d-axis voltage  */
	v->Vsq_ref = 0;			/* reference stator q-axis voltage  */
	v->Vsd_comp = 0;
	v->Vsq_comp = 0;
	v->k_comp = 0;//1.0 = 0;
//-------------------------------------------------//
// for the flux weakening
	v->speed_rated = 0;
	v->speed_ref_abs = 0;
	v->isd_virtual = 0;
	v->isd_ref_flux = 0;
	v->flux_ref = 0;
	v->flux_level = 0;
//-------------------------------------------------//
// for vvvf control
	v->target_value = 0;
	v->setpt_value = 0;
	v->rmp_dly_max = 0;
	v->rmp_delay_cntl = 0;
	v->step_angle = 0;
	v->angle_rg = 0;


fe1.Sensor_meca = 0;// if there is speed sensor
//------------------------------------------------------------//					
fe1.LrbyLm = 0;			/* Parameter:Lr divided by Lm (PU) */
fe1.Lsig = 0;			/* Parameter:Lsig = Ls*[1-Lm^2/(Ls*Lr)] (PU) */
fe1.Rs = 0;				/* Parameter:Stator resistance (PU) */
fe1.flux_ref = 0;
fe1.Lm = 0;				/* Parameter:Magnetization inductance (PU) */
fe1.Tr_invt = 0;		/* Parameter:Cut frequency for current model (PU) */
fe1.tc = 0;				/* Parameter:Sampling period (PU) */
//------------------------------------------------------------//					
fe1.is_al = 0;			/* Input:Stationary alfa-axis stator current (PU) */
fe1.is_be = 0;			/* Input:Stationary beta-axis stator current (PU) */
fe1.is_al_old = 0;		/* Variable:Stationary alfa-axis stator current of the last cycle (PU) */
fe1.is_be_old = 0;		/* Variable:Stationary beta-axis stator current of the last cycle (PU) */					
fe1.us_al = 0;			/* Input:Stationary alfa-axis stator voltage (PU) */
fe1.us_be = 0;			/* Input:Stationary beta-axis stator voltage (PU) */
fe1.deta_al = 0;
fe1.deta_be = 0;
fe1.isd_ref = 0;		/* Input:Reference d-axis stator current (PU) */
fe1.isq_ref = 0;
fe1.isd_fe = 0;
fe1.isq_fe = 0;
//------------------------------------------------------------//					
fe1.er_al = 0;			/* Output:alfa-axis rotor opposing electromotive force (PU) */
fe1.er_be = 0;			/* Output:beta-axis rotor opposing electromotive force (PU) */
				 
fe1.flux_al_v_fe = 0;	/* Variable:Alfa-axis rotor flux in voltage model (PU) */
fe1.flux_be_v_fe = 0;	/* Variable:Beta-axis rotor flux in voltage model (PU) */
fe1.flux_v_amp = 0;
fe1.flux_al_v_fe2 = 0;	
fe1.flux_be_v_fe2 = 0;	
fe1.flux_v_amp2 = 0;
//-------------------------------------------------------------//	
fe1.flux_al_i_fe = 0;	/* Variable:Alfa-axis rotor flux in current model (PU) */
fe1.flux_be_i_fe = 0;	/* Variable:Beta-axis rotor flux in current model (PU) */
fe1.flux_r_i_fe = 0;	/* Variable:Rotor flux amplitude in current model (PU) */
fe1.flux_r_i_fe_old = 0;
fe1.theta_r_i_fe = 0;	
//-------------------------------------------------------------//	

fe1.theta_r_fe = 0;		/* Output:Rotor flux angle */
fe1.amp_r_fe = 0;		/* Output:Rotor flux amplitude (PU) */
fe1.amp_r_old = 0;		/* Output:Last period rotor flux amplitude (PU) */
fe1.flux_r_2 = 0;		/* Variable:Squre of rotor flux amplitude (PU) */
fe1.flux_al_fe = 0;	/* Output:alfa-axis rotor flux (PU) */
fe1.flux_be_fe = 0;	/* Output:beta-axis rotor flux (PU) */
fe1.flux_al_fe_old = 0;	/* Output:alfa-axis rotor flux (PU) */
fe1.flux_be_fe_old = 0;	/* Output:beta-axis rotor flux (PU) */
//-------------------------------------------------------------//	

fe1.wc = 0;
fe1.wc_syn = 0;
fe1.w_syn = 0;			/* Output:Synchronous speed (PU) */
fe1.w_syn_flt = 0;
fe1.w_slip = 0;			/* Variable:Slippage frequency (PU) */
fe1.w_slip_old = 0;
fe1.w_slip_flt = 0;	
fe1.slip_coe = 0;
fe1.wr_elec_es = 0;		/* Output:Rotor speed (PU) */
fe1.wr_elec_es_flt = 0;
fe1.wr_elec_sensor = 0;

	
}

#pragma CODE_SECTION(Vector_ctrl_calc, "ramfuncs")
void Vector_ctrl_calc(Vector_ctrl *v)
{
//=======================================================================================//
	PARK park1 = PARK_DEFAULTS;
	IPARK ipark1 = IPARK_DEFAULTS;
	IPARK ipark2 = IPARK_DEFAULTS;
	ICLARKE iclarke1 = ICLARKE_DEFAULTS;
	ICLARKE iclarke2 = ICLARKE_DEFAULTS;
	CLARKE clarke1 = CLARKE_DEFAULTS;

//转子位置速度检测
	v->speed_ref = v->speed_ref_Hz*DPI/para.BASE_w;		//50Hz转速标幺化到1
	v->wr_meca =  v->wr_meca_SI*para2.p/para.BASE_w;	//机械角速度标幺化
	v->wr_elec =  v->wr_meca;							//机械角速度和电角速度标幺化后相同
	v->posi_elec = v->posi_meca*para2.p;				//转子电气位置

	while (v->posi_elec > DPI)
		v->posi_elec -= DPI;

	if(v->speed_ref>para2.Freq_max)						//转速限制
		v->speed_ref = para2.Freq_max;
	if(v->speed_ref<(-para2.Freq_max))
		v->speed_ref = -para2.Freq_max;

//参考转速滤波
	lpf_w_ref.x = v->speed_ref;
	lpf_w_ref.calc(&lpf_w_ref);
	v->speed_ref_flt = lpf_w_ref.y;
//=======================================================================================//
	v->Isa = v->Isa_SI*para.BASE_I_inv;					//三相电流标幺化
	v->Isb = v->Isb_SI*para.BASE_I_inv;
	v->Isc = v->Isc_SI*para.BASE_I_inv;
//电流派克变换
	v->sin_theta = sin(v->flux_theta);
	v->cos_theta = cos(v->flux_theta);

	clarke1.as = v->Isa;
	clarke1.bs = v->Isb;
	clarke1.cs = v->Isc;
 	clarke1.calc(&clarke1);
	v->ISA = clarke1.xalpha;
	v->ISB = clarke1.xbeta;

	park1.xalpha = v->ISA;
	park1.xbeta = v->ISB;
	park1.ang = v->flux_theta;
	park1.sin_ang = v->sin_theta;
	park1.cos_ang = v->cos_theta;
 	park1.calc(&park1);
//---------------------------------------//
//电流的同步滤波及死区补偿
	lpf_isd.x = park1.xd;
	lpf_isq.x = park1.xq;
	lpf_isd.calc(&lpf_isd);
	lpf_isq.calc(&lpf_isq);
	
	ipark2.xd = lpf_isd.y;
	ipark2.xq = lpf_isq.y;
	ipark2.ang = v->flux_theta;
	ipark2.sin_ang = v->sin_theta;
	ipark2.cos_ang = v->cos_theta;
	ipark2.calc(&ipark2);
	
	iclarke1.xalpha = ipark2.xalpha;
	iclarke1.xbeta = ipark2.xbeta;
	iclarke1.calc(&iclarke1);
	v->Isa_flt = iclarke1.as;
	v->Isb_flt = iclarke1.bs;
	v->Isc_flt = iclarke1.cs;
//===========================================异步电机矢量控制===============================================//
//---------------------------------------------------------------//
//转速和位置闭环
	if(v->speed_pi_count>10)			//每10个周期计算一次PI
	{
		pi_speed.pi_fdb = v->speedback;//pu--->Hz
		pi_speed.pi_ref = v->speed_ref_flt;
		pi_speed.calc(&pi_speed);
		v->Te_ref = pi_speed.pi_out;
		v->speed_pi_count = 0;
	}
	v->speed_pi_count++;
//---------------------------------------------------------------//		 
//dq轴电流给定以及预励磁
		v->flux_ref = para2.flux_ref*v->flux_level;
		v->isd_ref = v->flux_ref/para2.Lm;
		if (v->flux_delay_cnt < 4000)// 2s
		{
			v->isq_ref = 0.0;
			pi_speed.ui = 0.0;
			v->flux_delay_cnt++;
		}
		else
			//v->isq_ref = v->Te_ref/(para2.p*para2.LmbyLr*fe1.amp_r_fe);
			v->isq_ref = v->Te_ref/(para2.p*para2.LmbyLr*fe1.flux_ref);
//--------------------------------------------------------------//	
//定子d轴电流闭环
		pi_isd.pi_fdb = park1.xd;
		pi_isd.pi_ref = v->isd_ref;
		pi_isd.calc(&pi_isd);
		v->Vsd_ref = pi_isd.pi_out;
//-------------------------------------------------------------//	
//定子q轴电流闭环
		pi_isq.pi_fdb = park1.xq;
		pi_isq.pi_ref = v->isq_ref;
		pi_isq.calc(&pi_isq);
		v->Vsq_ref = pi_isq.pi_out;
//-------------------------------------------------------------//
//dq轴输出电压计算
		v->Vsd_comp = -fe1.w_syn_flt*para2.Lsig*park1.xq + para2.LmbyLr*(fe1.amp_r_fe -fe1.amp_r_old)/para2.Ts;
		v->Vsq_comp =  fe1.w_syn_flt*para2.Lsig*park1.xd + para2.LmbyLr*fe1.amp_r_fe*fe1.w_syn_flt;
//		Vsd_comp = -speedback*para2.Lq*isq_ref;//0;//
//		Vsq_comp = speedback*para2.flux_d + speedback*para2.Ld*isd_ref;//;//
		v->k_comp = 0.0;
		v->Vsd_ref = v->Vsd_ref + v->k_comp*v->Vsd_comp;
		v->Vsq_ref = v->Vsq_ref + v->k_comp*v->Vsq_comp;

//dq轴输出电压限幅
		if (v->Vsd_ref > para2.Udq_max)
			v->Vsd_ref = para2.Udq_max;
		if (v->Vsd_ref < -para2.Udq_max)
			v->Vsd_ref = -para2.Udq_max;
		if (v->Vsq_ref > para2.Udq_max)
			v->Vsq_ref = para2.Udq_max;
		if (v->Vsq_ref < -para2.Udq_max)
			v->Vsq_ref = -para2.Udq_max;
//=======================================================================================//
//PI控制器初始化
		if (v->start ==0)
		{
			pi_speed.ui = 0.0;
			pi_isd.ui = 0.0;
			pi_isq.ui = 0.0;
			v->Vsd_ref = 0.0;
			v->Vsq_ref = 0.0;
			v->flux_delay_cnt = 0;
			v->speed_pi_count = 0;
		}
//=======================================================================================//
//磁链观测(flux estimation)算法的输入量
	fe1.isd_fe = park1.xd;
	fe1.isq_fe = park1.xq;
	fe1.isd_ref = v->isd_ref;
	fe1.isq_ref = v->isq_ref;
	fe1.is_al = v->ISA;
	fe1.is_be = v->ISB;
	fe1.us_al = v->Ualfa_ref_pu;//svm1.Ualfa;//
	fe1.us_be = v->Ubeta_ref_pu;//svm1.Ubeta;//
	fe1.flux_ref = v->flux_ref;
	fe1.wr_elec_sensor = v->wr_elec;
//	fe1.calc(&fe1);
//==============================================================//
//有速度传感器
if(v->control_method == 1)// with  sensor
{
	fe1.Sensor_meca = 1;
	fe1.wr_elec_sensor = v->wr_elec;////////////
	fe1.calc(&fe1);
	v->speedback = v->wr_elec;
	v->flux_theta = fe1.theta_r_fe;
	lpf_w.x = v->speedback;
	lpf_w.calc(&lpf_w);
	v->speedback = lpf_w.y;

}
//==============================================================//
//无速度传感器
else if (v->control_method == 0)// without sensor
{
	fe1.Sensor_meca = 0;// without sensor
	fe1.calc(&fe1);
	v->speedback = fe1.wr_elec_es;	//观测的转速
	v->flux_theta = fe1.theta_r_fe;	//观测的磁链角度
	lpf_w.x = v->speedback;			//观测转速滤波
	lpf_w.calc(&lpf_w);
	v->speedback = lpf_w.y;
}
//==============================================================//
// VVVF	
else if (v->control_method == 2)
{
	fe1.Sensor_meca = 1;
	fe1.calc(&fe1);
//-------------------------------------------//
//电压频率设定
	v->target_value = v->speed_ref;//pu value
      if (((v->target_value - v->setpt_value)> para2.VVVF_step_freq)||((v->target_value - v->setpt_value)<(-para2.VVVF_step_freq) ))
      {
         v->rmp_delay_cntl += 1;
         if (v->rmp_delay_cntl >= v->rmp_dly_max)
         {
           v->rmp_delay_cntl = 0;
           if (v->target_value > v->setpt_value)
           {
            v->setpt_value += para2.VVVF_step_freq; 
           }
           else
           {
            v->setpt_value -= para2.VVVF_step_freq;  
         
           }
         }
        }
//--------------------------------------------//  
//电压角度设定               
		v->step_angle = v->setpt_value*para2.Ts;
        v->angle_rg += v->step_angle;      
      
        if (v->angle_rg> DPI)
          v->angle_rg -= DPI;
        if (v->angle_rg< 0.0)
          v->angle_rg += DPI;
//-------------------------------------------//                
//输出电压以及磁链角度
		v->Vsd_ref = v->setpt_value*para2.VVVF_base_volt;
		v->Vsq_ref = 0.0;
		v->flux_theta = v->angle_rg;
//		v->sin_theta = sin(v->angle_rg);
//		v->cos_theta = cos(v->angle_rg);	
}
else
{

}

//=======================================================================================//
	if(v->flux_theta>(1.0*DPI))
		v->flux_theta -= (1.0*DPI);
	if(v->flux_theta<0.0)
		v->flux_theta += (1.0*DPI);

	v->sin_theta = sin(v->flux_theta);
	v->cos_theta = cos(v->flux_theta);
//=======================================================================================//
//计算三相输出电压参考值
	ipark1.sin_ang = v->sin_theta;//sin(flux_theta);
	ipark1.cos_ang = v->cos_theta;//cos(flux_theta);
    ipark1.xd = v->Vsd_ref;
    ipark1.xq = v->Vsq_ref;
    ipark1.calc(&ipark1);

	v->Ualfa_ref_pu = ipark1.xalpha;
	v->Ubeta_ref_pu = ipark1.xbeta;
	iclarke2.xalpha = ipark1.xalpha;
	iclarke2.xbeta = ipark1.xbeta;
	iclarke2.calc(&iclarke2);
	v->Ua_ref = iclarke2.as*para.BASE_U;
	v->Ub_ref = iclarke2.bs*para.BASE_U;
	v->Uc_ref = iclarke2.cs*para.BASE_U;
	v->Ualfa_ref = v->Ualfa_ref_pu*para.BASE_U;
	v->Ubeta_ref = v->Ubeta_ref_pu*para.BASE_U;
}

//======================================================================================================================================//
#pragma CODE_SECTION(flux_estimate, "ramfuncs")
void flux_estimate(FE *v)
{
//----------------------------------------------------//
	v->flux_al_fe_old = v->flux_al_fe;		//观测alpha轴磁链
	v->flux_be_fe_old = v->flux_be_fe;		//观测beta轴磁链
	v->amp_r_old = v->amp_r_fe;				//观测磁链幅值
//current model in d-q axes
	v->flux_r_i_fe_old = v->flux_r_i_fe;
//转子d轴磁链计算
//	v->flux_r_i_fe = v->flux_r_i_fe + (v->tc*v->Tr_invt)*(v->Lm*v->isd_ref  -v->flux_r_i_fe);
	v->flux_r_i_fe = v->flux_r_i_fe + (v->tc*v->Tr_invt)*(v->Lm*v->isd_fe  -v->flux_r_i_fe);
//------------------------------------------------------//

if (v->Sensor_meca ==1) // control with speed sensor
{
	v->amp_r_fe = v->flux_r_i_fe;
	if(v->amp_r_fe > 0.0001)//  modified by zheng in 2012.3.21
		v->w_slip = v->Lm*v->Tr_invt*v->isq_fe/v->amp_r_fe;
	else
		v->w_slip = 0.0;
				
	v->w_slip = v->w_slip*v->slip_coe;
	v->w_slip_flt += v->tc*v->wc*(v->w_slip - v->w_slip_flt);		
	v->w_syn = v->wr_elec_sensor + v->w_slip;
	v->w_syn_flt += v->tc*v->wc*(v->w_syn - v->w_syn_flt);	
	v->theta_r_i_fe += v->w_syn_flt*v->tc;//0~2*pi
	if (v->theta_r_i_fe >= DPI )
		v->theta_r_i_fe = v->theta_r_i_fe  - DPI;
	if (v->theta_r_i_fe <0.0 )
		v->theta_r_i_fe = v->theta_r_i_fe  + DPI;
	
	v->theta_r_fe = v->theta_r_i_fe;
//-------------------------------------------------------//
//	无传感器算法的强制同步

	lpf_al_er.y = 0;
	lpf_be_er.y = 0;
	lpfpf_al_er.y = 0;
	lpfpf_be_er.y = 0;
	lpfpfpf_al_er.y = 0;
	lpfpfpf_be_er.y = 0;
	v->er_al = 0;
	v->er_be = 0;

	v->flux_al_fe =  v->amp_r_fe*cos(v->theta_r_fe);
	v->flux_be_fe =  v->amp_r_fe*sin(v->theta_r_fe);
	lpfi_al.y = v->flux_al_fe;
	lpfi_be.y = v->flux_be_fe;
	v->flux_al_v_fe = v->flux_al_fe;
	v->flux_be_v_fe = v->flux_be_fe;
	v->flux_v_amp = v->amp_r_fe;

	lpf_al_fe_i.y = v->flux_al_fe;
	lpf_be_fe_i.y = v->flux_be_fe;

	v->flux_al_i_fe = lpf_al_fe_i.y;
	v->flux_be_i_fe = lpf_be_fe_i.y;
	v->wr_elec_es = v->wr_elec_sensor;
	v->wr_elec_es_flt = v->wr_elec_sensor;
	
//----------------------------------------------------------//	

}
//===============================================================================//

else  //without speed sensor
{

//===============================================================================//
	v->deta_al = v->is_al - v->is_al_old;
	v->deta_be = v->is_be - v->is_be_old;
	/* Rotor opposing electromotive force computation */
	v->er_al = v->LrbyLm*(v->us_al - v->is_al*v->Rs - v->Lsig*v->deta_al/v->tc);
	v->er_be = v->LrbyLm*(v->us_be - v->is_be*v->Rs - v->Lsig*v->deta_be/v->tc);
	/* Backup the stator current of the last cycle */
	/*转子反电势去零漂*/
	lpf_al_er.x = v->er_al;
	lpf_be_er.x = v->er_be;
	lpf_al_er.calc(&lpf_al_er);
	lpf_be_er.calc(&lpf_be_er);

	lpfpf_al_er.x = lpf_al_er.y;
	lpfpf_be_er.x = lpf_be_er.y;
	lpfpf_al_er.calc(&lpfpf_al_er);
	lpfpf_be_er.calc(&lpfpf_be_er);
	
	lpfpfpf_al_er.x = lpfpf_al_er.y;
	lpfpfpf_be_er.x = lpfpf_be_er.y;
	lpfpfpf_al_er.calc(&lpfpfpf_al_er);
	lpfpfpf_be_er.calc(&lpfpfpf_be_er);

	v->er_al = v->er_al - lpfpfpf_al_er.y;
	v->er_be = v->er_be - lpfpfpf_be_er.y;

	/*电压模型转子磁通计算，一阶惯性环节*/
//纯积分环节和高通滤波环节合并为一阶惯性环节
	lpfi_al.x = v->er_al;
	lpfi_be.x = v->er_be;
	lpfi_al.calc(&lpfi_al);
	lpfi_be.calc(&lpfi_be);
	v->flux_al_v_fe = lpfi_al.y;//- _IQmpy(derive1.LrbyLm,_IQmpy(derive1.Lsig,isa_lpf));// - lpfpfpf_al_er.y;
	v->flux_be_v_fe = lpfi_be.y;//- _IQmpy(derive1.LrbyLm,_IQmpy(derive1.Lsig,isb_lpf));// - lpfpfpf_be_er.y;	
	v->flux_v_amp = sqrt(v->flux_al_v_fe*v->flux_al_v_fe+ v->flux_be_v_fe*v->flux_be_v_fe);

	//-------------------------------//just for test
//没有经过高通滤波的电压模型磁链
	v->flux_al_v_fe2 += v->er_al*v->tc;
	v->flux_be_v_fe2 += v->er_be*v->tc;
	v->flux_v_amp2 = sqrt(v->flux_al_v_fe2*v->flux_al_v_fe2+ v->flux_be_v_fe2*v->flux_be_v_fe2);
//-------------------------------//
	//电流模型转子磁链进行低通滤波//
	lpf_al_fe_i.x = v->flux_r_i_fe*(cos(v->theta_r_fe));
	lpf_be_fe_i.x = v->flux_r_i_fe*(sin(v->theta_r_fe));
	lpf_al_fe_i.calc(&lpf_al_fe_i);
	lpf_be_fe_i.calc(&lpf_be_fe_i);
	v->flux_al_i_fe = lpf_al_fe_i.y;
	v->flux_be_i_fe = lpf_be_fe_i.y;
	//转子磁通计算，两个模型的结果相加//
	//

	v->flux_al_fe = v->flux_al_v_fe + v->flux_al_i_fe;
	v->flux_be_fe = v->flux_be_v_fe + v->flux_be_i_fe;
	v->flux_r_2 = v->flux_al_fe*v->flux_al_fe + v->flux_be_fe*v->flux_be_fe;
	v->amp_r_fe = sqrt(v->flux_r_2);
	//----------------------------------------------------------------------//

//speed estimation
	
	if ( v->amp_r_fe <= 0.0000001)
	{
		v->theta_r_fe = 0.0;
		v->w_syn = 0.0;
		v->w_slip = 0.0;

	}
	else
	{
		v->theta_r_fe = atan2(v->flux_be_fe,v->flux_al_fe );// in the range of -pi ~pi

		v->w_syn = (v->flux_be_fe*v->flux_al_fe_old -v->flux_al_fe*v->flux_be_fe_old)/(v->flux_r_2*v->tc);
		v->w_slip = v->Lm* v->Tr_invt*v->isq_fe/v->amp_r_fe;		
		v->w_slip = v->w_slip*v->slip_coe;

	}
	
	
	v->w_syn_flt += v->tc*v->wc*(v->w_syn - v->w_syn_flt);
	v->w_slip_flt += v->tc*v->wc*(v->w_slip - v->w_slip_flt);
			
	v->wr_elec_es = v->w_syn - v->w_slip;
	v->wr_elec_es_flt += v->tc*v->wc*(v->wr_elec_es - v->wr_elec_es_flt);

	//----------------------------------------------------------------------//

}
//===============================================================================//
	v->is_al_old = v->is_al;////////////////////////
	v->is_be_old = v->is_be;///////////////////////

}
//======================================================================================================================================//
void Para_derive(Parameter_set *p)
{
//-----------------------------------------//
//-----------------------------------------//

	p->UP_N = p->UL_N/1.732;
	p->wmeca_N = DPI*p->rpm_N/60.0;
	p->ws_N = DPI*p->F_N;//同步角速度
	p->T_N = p->P_N/p->wmeca_N;
	p->flux_r_N = p->UP_N*1.4142136/p->ws_N;
	p->flux_ref = p->flux_r_N*0.9;
	p->flux_ref_dq = p->flux_ref*1.0;
//-----------------------------------------//
	p->Umax = p->UP_N*1.414;
	p->Udq_max = p->UP_N*1.414*1.0;//1.414*1.0;//1.2247;
//	p->Idq_max = p->I_N*1.414*1.2247*2.0;
	p->Imax = p->I_N*1.414*1.0;//2.0对应允许过流倍数
	p->Idq_max = p->Imax*1.0;
	p->Torque_max = p->T_N*1.0;	
	p->Freq_max = p->F_N*1.0;
	p->flux_d = p->flux_r*1.0;// for PMSM
	p->flux_max = p->flux_r_N;
	p->L1 = p->Ls - p->Lm;
	p->L2 = p->Lr - p->Lm;
//---------------------------//	
	p->BASE_U = p->UP_N*1.414;
	p->BASE_I = p->I_N*1.414;
	p->BASE_Power = p->BASE_U*p->BASE_I;
	p->BASE_FREQ = p->F_N;
	p->BASE_w = DPI*p->BASE_FREQ;
	p->BASE_time = 1.0/p->BASE_w;
	p->BASE_Torque = p->BASE_Power/p->BASE_w;
	p->BASE_Flux = p->BASE_U*p->BASE_time;
	p->BASE_R = p->BASE_U/p->BASE_I;
	p->BASE_L = p->BASE_R*p->BASE_time;
	p->BASE_Interia = p->BASE_Torque*p->BASE_time/p->BASE_w;
	p->BASE_I_inv = 1.0/p->BASE_I;
	p->BASE_U_inv = 1.0/p->BASE_U;
	//---------------------------//	
	p->rs = p->Rs/p->BASE_R;
	p->rr = p->Rr/p->BASE_R;
	p->ls = p->Ls/p->BASE_L;
	p->lr = p->Lr/p->BASE_L;
	p->lm = p->Lm/p->BASE_L;
	p->ld = p->Ld/p->BASE_L;
	p->lq = p->Lq/p->BASE_L;
	p->l1 = p->L1/p->BASE_L;
	p->l2 = p->L2/p->BASE_L;
	p->flux_d_pu = p->flux_d/p->BASE_Flux;
	p->flux_r_pu = p->flux_r/p->BASE_Flux;
	p->flux_ref_pu = p->flux_ref/p->BASE_Flux;
	p->flux_ref_dq_pu = p->flux_ref_dq/p->BASE_Flux;
	p->inertia_pu = p->Inertia/p->BASE_Interia;
	//p->friction_pu;
	p->wc_wr_pu = p->Wc_wr/p->BASE_w;
	p->wc_cur_pu = p->Wc_cur/p->BASE_w;
	p->wc_offset_pu = p->Wc_offset/p->BASE_w;
	p->Umax_pu = p->Umax/p->BASE_U;
	p->Imax_pu = p->Imax/p->BASE_I;
	p->Udq_max_pu = p->Udq_max/p->BASE_U;
	p->Idq_max_pu = p->Idq_max/p->BASE_I;
	p->Torque_max_pu = p->Torque_max/p->BASE_Torque;
	p->flux_max_pu = p->flux_max/p->BASE_Flux; 
	p->ts = p->Ts/p->BASE_time;
	p->freq_max_pu = p->Freq_max/p->BASE_FREQ;

}


//======================================================================================================================================//
#pragma CODE_SECTION(pi_speed_const_calc, "ramfuncs")
void pi_speed_const_calc(PI_SPEED_CONST *v)
{	
	float taun = v->h/v->wc_wr;
    v->Kp =  (v->h+1.0)/(2.0*taun)*v->inertia;
    v->Ki = v->Kp/taun;
}
//======================================================================================================================================//
#pragma CODE_SECTION(pi_current_const_calc, "ramfuncs")
void pi_current_const_calc(PI_CURRENT_CONST *v)
{	
   v->Kp = v->wc_current*v->ls;
   v->Ki = v->wc_current*v->rs;       //常规的方法
}


//======================================================================================================================================//
#pragma CODE_SECTION(pi_speed_calc, "ramfuncs")
void pi_speed_calc(PI_SPEED *v)
{	
	v->err = v->pi_ref - v->pi_fdb;

	/* Integral segment */
	v->ui_delta = v->err*v->Ki*v->Tc;

	v->ui += v->ui_delta;
	
	/* Proportional segment */
	v->up = v->Kp*v->err;

	/* Proportional and integral repeated addition */
	v->pi_out = v->ui + v->up;
	if ( v->pi_out > v->pi_out_max )
	{	v->pi_out = v->pi_out_max;
		v->ui -= v->ui_delta;			/* Stop integral when saturation */
	}
	else if ( v->pi_out < -v->pi_out_max )
	{	v->pi_out = -v->pi_out_max;
		v->ui -= v->ui_delta;			/* Stop integral when saturation */
	}
}
//======================================================================================================================================//
#pragma CODE_SECTION(pi_fun_calc, "ramfuncs")
void pi_fun_calc(PI_fun *v)
{	
	v->err = v->pi_ref - v->pi_fdb;

	/* Integral segment */
	v->ui_delta = v->Tc*v->Ki*v->err;
/*
	if ( v->ui_delta > v->pi_var_max )	v->ui_delta = v->pi_var_max;
	else if (v->ui_delta < -v->pi_var_max )	v->ui_delta = -v->pi_var_max;
*/	
	v->ui += v->ui_delta;
	
	/* Proportional segment */
	v->up = v->Kp*v->err;
/*
	if ( v->up > v->pi_var_max)		v->up = v->pi_var_max;
	else if ( v->up< -v->pi_var_max )	v->up = -v->pi_var_max;
*/	
	/* Proportional and integral repeated addition */
	v->pi_out = v->ui + v->up;

	if ( v->pi_out > v->pi_out_max )
	{	v->pi_out = v->pi_out_max;
		v->ui -= v->ui_delta;			/* Stop integral when saturation */
	}
	else if ( v->pi_out < -v->pi_out_max )
	{	v->pi_out = -v->pi_out_max;
		v->ui -= v->ui_delta;			/* Stop integral when saturation */
	}
}
//======================================================================================================================================//
#pragma CODE_SECTION(pi_flux_calc, "ramfuncs")
void pi_flux_calc(PI_flux *v)
{	
	v->err = v->pi_ref - v->pi_fdb;

	/* Integral segment */
	v->ui_delta = v->Tc*v->Ki*v->err;
/*
	if ( v->ui_delta > v->pi_var_max )	v->ui_delta = v->pi_var_max;
	else if (v->ui_delta < -v->pi_var_max )	v->ui_delta = -v->pi_var_max;
*/	
	v->ui += v->ui_delta;
	
	/* Proportional segment */
	v->up = v->Kp*v->err;
/*
	if ( v->up > v->pi_var_max)		v->up = v->pi_var_max;
	else if ( v->up< -v->pi_var_max )	v->up = -v->pi_var_max;
*/	
	/* Proportional and integral repeated addition */
	v->pi_out = v->ui + v->up;

	if ( v->pi_out > v->pi_out_max )
	{	v->pi_out = v->pi_out_max;
		v->ui -= v->ui_delta;			/* Stop integral when saturation */
	}
	else if ( v->pi_out < v->pi_out_min )
	{	v->pi_out = v->pi_out_min;
		v->ui -= v->ui_delta;			/* Stop integral when saturation */
	}
}
//==================================================================================//

//======================================================================================================================================//






