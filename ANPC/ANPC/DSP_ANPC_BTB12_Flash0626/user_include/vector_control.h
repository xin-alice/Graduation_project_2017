
#define PI  3.14159265359
#define DPI 6.2831853072

//======================================================================================================================================//
// main program code of the vector control of induction motor
typedef struct {  	
//-------------------------------------------------//
	unsigned int control_method;// if 1 there is sensor, 0 without sensor,3 VVVF
	unsigned int flux_delay_cnt;// to build the rotor flux
	unsigned int start;
//-------------------------------------------------//
	float test1;
	float test2;
	float test3;
	float test4;
//-------------------------------------------------//

//input
	float Isa_SI;
	float Isb_SI;
	float Isc_SI;
	float wr_meca_SI;
	float posi_meca;           /* Speed reference given by PC scope */
	float posi_elec;
	float posi_ab;//转子的绝对位置
	float speed_ref_Hz;
	float speed_ref;           /* Speed reference  */
	float speed_ref_flt;				 /* Speed reference after filter (pu) */
	float  posi_ref;
//-------------------------------------------------//
//output
	float Ualfa_ref;
	float Ubeta_ref;
	float Ualfa_ref_pu;
	float Ubeta_ref_pu;
	float Ua_ref;
	float Ub_ref;
	float Uc_ref;
//-------------------------------------------------//
	float wr_elec;
	float wr_meca;
//-------------------------------------------------//
	float Isa;
	float Isb;
	float Isc;
	float speedback;
	float Isa_flt;
	float Isb_flt;
	float Isc_flt;
//-------------------------------------------------//				
	int speed_pi_count;
	int posi_pi_count;
	float 	p_posi_coe;//0.1;
	float   i_posi_coe;
	float	p_speed_coe;//1.0;	//3	
	float	i_speed_coe;//0.01;
	float	p_isd_coe;		
	float	i_isd_coe;		
	float   p_isq_coe;		
	float	i_isq_coe;
	float   p_flux_coe;
	float   i_flux_coe;
//--------------------------------------------------//		
	float	ISA;
	float	ISB;
	float flux_theta;
	float cos_theta;
	float sin_theta;
	float	Te_ref;				/* reference electromagnetic torque  */
	float	isd_ref;			/* reference stator d-axis current  */
	float	isq_ref;			/* reference stator q-axis current  */
	float	Vsd_ref;			/* reference stator d-axis voltage  */
	float	Vsq_ref;			/* reference stator q-axis voltage  */
	float   Vsd_comp;
	float   Vsq_comp;
	float   k_comp;//1.0;
//-------------------------------------------------//
// for the flux weakening
	float speed_rated;
	float speed_ref_abs;
	float isd_virtual;
	float isd_ref_flux;
	float flux_ref;
	float flux_level;
//-------------------------------------------------//
// for vvvf control
	float target_value;
	float setpt_value;
	unsigned int rmp_dly_max;
	unsigned int rmp_delay_cntl;
	float step_angle;
	float angle_rg;
//-------------------------------------------------//
	void  (*init)();
	void  (*reset)();
	void  (*calc)();	/* Pointer to calculation function */ 
	 } Vector_ctrl;	            

typedef Vector_ctrl *Vector_ctrl_handle;
//----------------------------------------------------------------//                   
#define Vector_ctrl_DEFAULTS {0,0,0,0,0,0,0,0,0,0,\
							  0,0,0,0,0,0,0,0,0,0, \
							  0,0,0,0,0,0,0,0,0,0, \
							  0,0,0,0,0,0,0,0,0,0, \
							  0,0,0,0,0,0,0,0,0,0, \
							  0,0,0,0,0,0,0,0,0,0, \
							  0,0,0,0,0,0,0,0,0,0,  \
							  0, \
							  (void (*)(long))Vector_ctrl_init, \
							  (void (*)(long))Vector_ctrl_reset, \
	 		                (void (*)(long))Vector_ctrl_calc }
//----------------------------------------------------------------//   
void Vector_ctrl_init(Vector_ctrl_handle); 
void Vector_ctrl_reset(Vector_ctrl_handle);
void Vector_ctrl_calc(Vector_ctrl_handle);
//======================================================================================================================================//
// flux estimation or observer of the induction motor
typedef struct {  	
//------------------------------------------------------------//					
					unsigned int Sensor_meca;// if there is speed sensor
//------------------------------------------------------------//					

					float	LrbyLm;			/* Parameter:Lr divided by Lm (PU) */
					float	Lsig;			/* Parameter:Lsig = Ls*[1-Lm^2/(Ls*Lr)] (PU) */
					float	Rs;				/* Parameter:Stator resistance (PU) */
					float   flux_ref;
					float	Lm;				/* Parameter:Magnetization inductance (PU) */
					float	Tr_invt;		/* Parameter:Cut frequency for current model (PU) */
					float	tc;				/* Parameter:Sampling period (PU) */
//------------------------------------------------------------//					
					float	is_al;			/* Input:Stationary alfa-axis stator current (PU) */
					float	is_be;			/* Input:Stationary beta-axis stator current (PU) */
					float	is_al_old;		/* Variable:Stationary alfa-axis stator current of the last cycle (PU) */
					float	is_be_old;		/* Variable:Stationary beta-axis stator current of the last cycle (PU) */					
					float   us_al;			/* Input:Stationary alfa-axis stator voltage (PU) */
					float	us_be;			/* Input:Stationary beta-axis stator voltage (PU) */
					float	deta_al;
					float	deta_be;
				 	float	isd_ref;		/* Input:Reference d-axis stator current (PU) */
					float	isq_ref;
					float	isd_fe;
					float   isq_fe;

//------------------------------------------------------------//					
					float	er_al;			/* Output:alfa-axis rotor opposing electromotive force (PU) */
					float	er_be;			/* Output:beta-axis rotor opposing electromotive force (PU) */
				 
					float	flux_al_v_fe;	/* Variable:Alfa-axis rotor flux in voltage model (PU) */
					float	flux_be_v_fe;	/* Variable:Beta-axis rotor flux in voltage model (PU) */
					float	flux_v_amp;
					float	flux_al_v_fe2;	
					float	flux_be_v_fe2;	
					float	flux_v_amp2;
//-------------------------------------------------------------//	
					float   flux_al_i_fe;	/* Variable:Alfa-axis rotor flux in current model (PU) */
					float	flux_be_i_fe;	/* Variable:Beta-axis rotor flux in current model (PU) */
					float	flux_r_i_fe;	/* Variable:Rotor flux amplitude in current model (PU) */
					float   flux_r_i_fe_old;
					float	theta_r_i_fe;	
//-------------------------------------------------------------//	

					float	theta_r_fe;		/* Output:Rotor flux angle */
					float	amp_r_fe;		/* Output:Rotor flux amplitude (PU) */
					float	amp_r_old;		/* Output:Last period rotor flux amplitude (PU) */
					float	flux_r_2;		/* Variable:Squre of rotor flux amplitude (PU) */
					float	flux_al_fe;	/* Output:alfa-axis rotor flux (PU) */
					float	flux_be_fe;	/* Output:beta-axis rotor flux (PU) */
					float	flux_al_fe_old;	/* Output:alfa-axis rotor flux (PU) */
					float	flux_be_fe_old;	/* Output:beta-axis rotor flux (PU) */
//-------------------------------------------------------------//	

					float   wc;
					float   wc_syn;
					float	w_syn;			/* Output:Synchronous speed (PU) */
					float	w_syn_flt;
					float	w_slip;			/* Variable:Slippage frequency (PU) */
					float	w_slip_old;
					float	w_slip_flt;	
					float   slip_coe;
					float	wr_elec_es;		/* Output:Rotor speed (PU) */
					float	wr_elec_es_flt;
					float   wr_elec_sensor;
					void  (*calc)();
				 } FE;	            

typedef FE *FE_handle;
//-------------------------------------------------------------//                   
#define FE_DEFAULTS {     0,0,0,0,0,0,0,0,0,0,\
	                      0,0,0,0,0,0,0,0,0,0,\
						  0,0,0,0,0,0,0,0,0,0,\
						  0,0,0,0,0,0,0,0,0,0,\
						  0,0,0,0,0,0,0,0,0,0,\
						  0,0,\
						  (void (*)(long))flux_estimate }

/*---------------------------------------------------------------------------*/
void flux_estimate(FE_handle);

//======================================================================================================================================//
typedef struct 
{
	//-------------------------//
	float Ts;
	float Fs_cpu_M; //150M Hz
	//-------------------------//
	float Rs;
	float Rr;
	float Rm;
	float Ls;
	float Lm;
	float Lr;
	float L1;
	float L2;
	float Ld;
	float Lq;
	unsigned int p; // poles paires
	float flux_d;
	float flux_r;
	float Inertia;
	float friction;
	float flux_ref;
	float flux_ref_dq;
	float flux_r_N;
	//-------------------------//
	float P_N;
	float UL_N;
	float UP_N;
	float I_N;
	float F_N;
	float rpm_N;
	float wmeca_N;
	float ws_N;//同步角速度
	float T_N;
	//-------------------------//
	float Wc_wr;
	float Wc_cur;
	float Wc_offset;
	
	unsigned int H;
	//-------------------------//
	float Imax;
	float Umax;
	float Udq_max;
	float Idq_max;
	float Torque_max;
	float flux_max;
	float Freq_max;
	//------------------------------------//
	//base values
	float BASE_U;
	float BASE_I;
	float BASE_R;
	float BASE_L;
	float BASE_Power;
	float BASE_w;
	float BASE_time;
	float BASE_FREQ;
	float BASE_Torque;
	float BASE_Flux;
	float BASE_Interia;
	float BASE_U_inv;
	float BASE_I_inv;
	//------------------------------------//
	//p.u. values
	float rs;
	float rr;
	float ls;
	float lr;
	float lm;
	float ld;
	float lq;
	float l1;
	float l2;
	float lsig;
	float flux_d_pu;
	float flux_r_pu;
	float inertia_pu;
	float friction_pu;
	float wc_wr_pu;
	float wc_cur_pu;
	float wc_offset_pu;
	float Imax_pu;
	float Umax_pu;
	float Udq_max_pu;
	float Idq_max_pu;
	float Torque_max_pu;
	float flux_max_pu;
	float freq_max_pu;
	float ts;
	float flux_ref_pu;
	float flux_ref_dq_pu;
	//------------------------------------//
	void  (*derive)();
} Parameter_set;

              			  
typedef Parameter_set *Parameter_set_handle;
//-------------------------------------------------------------------------------                     
#define Parameter_set_DEFAULTS {0,0,0,0,0,0,0,0,0,0,\
								0,0,0,0,0,0,0,0,0,0,\
								0,0,0,0,0,0,0,0,0,0,\
								0,0,0,0,0,0,0,0,0,0,\
								0,0,0,0,0,0,0,0,0,0,\
								0,0,0,0,0,0,0,0,0,0,\
                                0,0,0,0,0,0,0,0,0,0, \
								0,0,0,0,0,0,0,0,0,0, \
              			  (void (*)(long))Para_derive }

void Para_derive(Parameter_set_handle);

//============================================================================================//


typedef struct 
{
	float Rs;
	float Rr;
	float Ls;
	float Lm;
	float Lr;
	float Ld;
	float Lq;
	float LrbyLm;
	float LmbyLr;
	float Lsig;
	float Tr;
	float Tr_invt;
	unsigned int p; // poles paires
	float Ts;
	float flux_d;
	float flux_r;
	float flux_ref;
	float Inertia;
	float friction;
	float Wc_wr;
	float Wc_cur;
	float Wc_offset;
	float Imax;
	float Umax;
	float Udq_max;
	float Idq_max;
	float Torque_max;
	float flux_max;
	float Freq_max;
	//----------------------//
	float VVVF_step_freq;
	float VVVF_base_volt;
	//------------------------//
} Parameter_used;
//======================================================================================================================================//

typedef struct {  	float	wc_wr;			/* Input:Cut-off frequency for speed loop filter (PU) */
					float	inertia;	/* Input:Inertia (PU) */
					float	h;				/* Input:Bandwidth of the intermediate frequency */
					float	Ki;				/* Output:Integral gain */
					float	Kp;				/* Output:Proportional gain */
					void  (*calc)();	/* Pofloater to calculation function */ 
				 } PI_SPEED_CONST;	            

typedef PI_SPEED_CONST *PI_SPEED_CONST_handle;
//-------------------------------------------------------------//                     
#define PI_SPEED_CONST_DEFAULTS {0,0,0,0,0, \
			              (void (*)(long))pi_speed_const_calc }
//-------------------------------------------------------------//
void pi_speed_const_calc(PI_SPEED_CONST_handle);

/*******************************************************************************/

typedef struct {  	float	wc_current;		/* Input:Cut-off frequency for current loop filter (PU) */
					float	rr;				/* Input:Rotor resistance (PU) */
					float	rs;				/* Input:Stator resistance (PU) */
					float	lm;				/* Input:Flux inductance (PU) */
					float	lr;				/* Input:Rotor inductance (PU) */
					float	ls;				/* Input:Stator inductance (PU) */
					float	Ki;				/* Output:Integral gain */
					float	Kp;				/* Output:Proportional gain */
					void  (*calc)();	/* Pofloater to calculation function */ 
				 } PI_CURRENT_CONST;	            

typedef PI_CURRENT_CONST *PI_CURRENT_CONST_handle;
//-------------------------------------------------------------//                    
#define PI_CURRENT_CONST_DEFAULTS {0,0,0,0,0,0,0,0, \
			              (void (*)(long))pi_current_const_calc }
//-------------------------------------------------------------//
void pi_current_const_calc(PI_CURRENT_CONST_handle);


/*******************************************************************************/
typedef struct {  	float	pi_fdb;			/* Input:Feedback value */
					float	pi_ref;			/* Input:Reference value */
					float   Tc;				/* Parameter:sampleing period */
					float	Ki;				/* Parameter:Integral gain */
					float	Kp;				/* Parameter:Proportional gain */
					float	pi_out_max;		/* Parameter:Maximum output */
					float	err;			/* Variable:Input error */
					float	up;				/* Variable:Proportional output */
					float	ui;				/* Variable:Integral output */
					float	ui_delta;		/* Variable:Integtal increment */
					float	pi_out;			/* Output:PI output */
					int	pi_gain;		/* pi_gain determin the scale of pi */
					void  (*calc)();	/* Pofloater to calculation function */ 
				 } PI_SPEED;	            

typedef PI_SPEED *PI_SPEED_handle;
//-------------------------------------------------------------//                    
#define PI_SPEED_DEFAULTS {0,0,0,0,0,0,0,0,0,0,  \
                           0,0,  \
 		              (void (*)(long))pi_speed_calc }
//-------------------------------------------------------------//
void pi_speed_calc(PI_SPEED_handle);

/*******************************************************************************/

typedef struct {  	float	pi_fdb;			/* Input:Feedback value */
					float	pi_ref;			/* Input:Reference value */
					float   Tc;				/* Parameter:sampleing period */
					float	Ki;				/* Parameter:Integral gain */
					float	Kp;				/* Parameter:Proportional gain */
					float	pi_out_max;		/* Parameter:Maximum output */
					float	pi_var_max;		/* Parameter:Maximum variable */
					float	err;			/* Variable:Input error */
					float	up;				/* Variable:Proportional output */
					float	ui;				/* Variable:Integral output */
					float	ui_delta;		/* Variable:Integtal increment */
					float	pi_out;			/* Output:PI output */
					void  (*calc)();	/* Pofloater to calculation function */ 
				 } PI_fun;	            

typedef PI_fun *PI_fun_handle;
//-------------------------------------------------------------//                     
#define PI_DEFAULTS {0,0,0,0,0,0,0,0,0,0,  \
                     0,0,  \
	 		              (void (*)(long))pi_fun_calc }
//-------------------------------------------------------------//
void pi_fun_calc(PI_fun_handle);

/*******************************************************************************/

typedef struct {  	float	pi_fdb;			/* Input:Feedback value */
					float	pi_ref;			/* Input:Reference value */
					float   Tc;				/* Parameter:sampleing period */
					float	Ki;				/* Parameter:Integral gain */
					float	Kp;				/* Parameter:Proportional gain */
					float	pi_out_max;		/* Parameter:Maximum output */
					float	pi_out_min;		/* Parameter:Maximum variable */
					float	err;			/* Variable:Input error */
					float	up;				/* Variable:Proportional output */
					float	ui;				/* Variable:Integral output */
					float	ui_delta;		/* Variable:Integtal increment */
					float	pi_out;			/* Output:PI output */
					void  (*calc)();	/* Pofloater to calculation function */ 
				 } PI_flux;	            

typedef PI_flux *PI_flux_handle;
//-------------------------------------------------------------//                    
#define PI_flux_DEFAULTS {0,0,0,0,0,0,0,0,0,0,  \
                           0,0,  \
	 		              (void (*)(long))pi_flux_calc }
//-------------------------------------------------------------//
void pi_flux_calc(PI_flux_handle);


//======================================================================================================================================//

//============================================================================================//







