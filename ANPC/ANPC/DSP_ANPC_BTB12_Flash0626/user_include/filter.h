

typedef struct {  	_iq	x;				/* Input:Input of the low pass filter (PU) */
					_iq	Tc;				/* Parameter:Sampling period (PU) */
					_iq wc;				/* Parameter:Cut off frequency for low pass filter (PU) */
					_iq	y_old;			/* Variable:Output of the last cycle (PU) */
					_iq	y;				/* Output:Output of the low pass filter (PU) */
					_iq	lyh_scope1;
					_iq lyh_scope2;
					void  (*calc)();	/* Po_iqer to calculation function */ 
				 } LPF;

typedef LPF *LPF_handle;

/*-----------------------------------------------------------------------------
Default initalizer for the TIME_ER object.
-----------------------------------------------------------------------------*/                     
#define LPF_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,\
	                      0,\
	                      (void (*)(long))lpf_calc }
#define LPF_AL_F_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,\
	                      0,\
	                      (void (*)(long))lpf_calc }
#define LPF_BE_F_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,\
	                      0,\
	                      (void (*)(long))lpf_calc }
#define LPF_F_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,\
	                      0,\
	                      (void (*)(long))lpf_calc }	     
#define LPF_SYNW_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,\
	                      0,\
	                      (void (*)(long))lpf_calc }	 	                                    
#define LPF_W_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,\
	                      0,\
	                      (void (*)(long))lpf_calc }	 
/*------------------------------------------------------------------------------
Prototypes for the functions in TIME_LPF.C
------------------------------------------------------------------------------*/
void lpf_calc(LPF_handle);
/*******************************************************************************/

typedef struct {  	_iq	x;				/* Input:Input of the low pass filter (PU) */
					_iq	Tc;				/* Parameter:Sampling period (PU) */
					_iq wc;				/* Parameter:Cut off frequency for low pass filter (PU) */
					_iq	y_old;			/* Variable:Output of the last cycle (PU) */
					_iq	y;				/* Output:Output of the low pass filter (PU) */
					_iq	lyh_scope1;
					_iq lyh_scope2;
					void  (*calc)();	/* Po_iqer to calculation function */ 
				 } LPFI;	            

typedef LPFI *LPFI_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the TIME_LPFI object.
-----------------------------------------------------------------------------*/                     
#define LPFI_DEFAULTS {0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,	\
	                      0,	\
	 		              (void (*)(long))lpfi_calc }
#define LPFI_AL_DEFAULTS{0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	                      0,	\
	                      0,	\
	 		              (void (*)(long))lpfi_calc }
#define LPFI_BE_DEFAULTS{0,    /*  x  */  \
	                      0,    /*  Tc  */  \
	                      0,    /*  wc  */  \
	                      0,    /*  y_old */   \
	                      0,    /*  y  */ \
	 		              0,	\
	                      0,	\
	                      (void (*)(long))lpfi_calc }
/*------------------------------------------------------------------------------
Prototypes for the functions in TIME_ER.C
------------------------------------------------------------------------------*/
void lpfi_calc(LPFI_handle);


