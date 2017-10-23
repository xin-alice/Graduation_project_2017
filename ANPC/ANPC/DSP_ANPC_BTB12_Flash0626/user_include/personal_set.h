
#include "DSP2833x_Device.h"     // Headerfile Include File
#include "DSP2833x_Examples.h"   // Examples Include File


//===============================================================================//
#define    	ext_out        	(volatile unsigned int *)0x4001
#define		ext_in			(volatile unsigned int *)0x4000
//------------------------------------------------------------//
#define    	PWMa_enable     (volatile unsigned int *)0x4005
#define		PWMa_disable 	(volatile unsigned int *)0x4006
#define    	PWMb_enable     (volatile unsigned int *)0x4007
#define		PWMb_disable 	(volatile unsigned int *)0x4008
//#define		clr_Error_PWMa  (volatile unsigned int *)0x4009
//#define		Error_PWMa  	(volatile unsigned int *)0x400A
//#define		clr_Error_PWMb  (volatile unsigned int *)0x400B
//#define		Error_PWMb  	(volatile unsigned int *)0x400C
//==============================================================================//
//for new topology
#define    	PWMA_mode     	(volatile unsigned int *)0x4010
#define		PWMB_mode		(volatile unsigned int *)0x4011
#define		PWMC_mode		(volatile unsigned int *)0x4012
#define    	PWMU_mode     	(volatile unsigned int *)0x4013
#define		PWMV_mode		(volatile unsigned int *)0x4014
#define		PWMW_mode		(volatile unsigned int *)0x4015
#define		cpld_Ufa		(volatile unsigned int *)0x4016
#define		cpld_Ufb		(volatile unsigned int *)0x4017
#define		cpld_Ufc		(volatile unsigned int *)0x4018
#define		cpld_Ufu		(volatile unsigned int *)0x4019
#define		cpld_Ufv		(volatile unsigned int *)0x401a
#define		cpld_Ufw		(volatile unsigned int *)0x401b

#define		err_cpld			(volatile unsigned int *)0x4020

//===============================================================================//
#define PI  3.14159265359
#define DPI 6.2831853072
//#define PWM_period  37500
#define PWM_DBFED  600  // falling edge  4us
#define PWM_DBRED  600 // rising edge     4us



//-----------------------------------------------------=----*/

typedef struct {  float  Ta;  		/* Input: stationary d-axis stator variable */
				  float  Tb;		 /* Input: stationary q-axis stator variable */
				  float  Tc;		/* Input: rotating angle (pu) */

				  float  Tu;  		
				  float  Tv;		  
				  float  Tw;

				  int PWM_period;
		 	 	  void  (*Update)();	/* Pointer to calculation function */ 
				 } PWM_Update;	            

typedef PWM_Update *PWM_Update_handle;
//-----------------------------------------------------------------------------
//Default initalizer for the PARK object.
//-------------------------------------------------------------------------------                     
#define PWM_Update_DEFAULTS {  0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
						  0, \
              			  (void (*)(long))PWM_Update_calc }
void PWM_Update_calc(PWM_Update_handle);
//------------------------------------------------------------------------------
/*
typedef struct 
{
	int	Cur_u0;
	int Cur_v0;
	int	Cur_u;
	int	Cur_v;
	int	Cur_w;
	int Cur_u2;
	int Cur_v2;
//	int	Adc_0;
//	int	Cur_dc;
	int	Adc_1;
	int	Adc_2;
	int V_dc;
	int V_dc0;
	int V_dc2;
//------------//
	float	A;
	float	B;
	float Cur_u_ad;
	float Cur_v_ad;
	float Vdc_ad;
//---------------------------------------//
	float cur_param;
	float v_param;
	float Cur_u_actual;
	float Cur_v_actual;
    float VDC_actual;
//---------------------------------------//
	float	IA;
	float	IB;
	float	IC;
	float	VDC;
	float	VDCinv;
	
} Signal_ADC;
*/
//=========================================================================================//

//=========================================================================================//
//============================================================================================//

typedef struct
{	float Lm;
	float Ls;
	float Lr;
	float Rs;
	float Rr;
	float tao_r;
	float sigma;
	float pole;
	float J_motor;
	float torq;
	float uq_ref;
	float flux_ref;
	float UL;
	float IN;
	float PN;
	float FN;
} para_motor;

#define para_motor_DEFAULTS {0, \
							  0, \
							  0, \
	                          0, \
	                          0, \
	                          0, \
	                          0, \
							  0, \
							  0, \
							  0, \
							  0, \
							  0, \
							  0, \
							  0, \
							  0, \
							  0}

typedef struct
{	float Udc;
	float fs;
	float Tp;
	float encoder_linear;
	float Cd;
	float Cf;
	float Ls;
	float Imax;
	float Udmax;
	float Ucfmax;
} para_converter;

#define para_converter_DEFAULTS {0, \
							  0, \
							  0, \
							  0, \
	                          0, \
	                          0, \
	                          0, \
							  0, \
							  0, \
							  0}


void Personal_set();

