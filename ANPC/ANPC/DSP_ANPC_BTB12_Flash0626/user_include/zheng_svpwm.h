
typedef struct 	{ float  Ualfa; 			/* Input: reference alfa-axis phase voltage   */
				  float  Ubeta;			/* Input: reference beta-axis phase voltage   */
				  float  V_ba_inv;
				  int  sector;
				  float  Vdc;
				  float  T1;				
				  float  T2;		
				  float  T0;
				  float  T0_half;
				  float  Ta0;				/* Output: reference phase-a switching function  */			
				  float  Tb0;				/* Output: reference phase-b switching function   */
				  float  Tc0;				/* Output: reference phase-c switching function   */
				  float  Ta;				/* Output: reference phase-a switching function  */			
				  float  Tb;				/* Output: reference phase-b switching function   */
				  float  Tc;				/* Output: reference phase-c switching function   */
				  void   (*calc)();	    /* Pointer to calculation function */ 
				} zheng_svm;
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																				
typedef zheng_svm *zheng_svm_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SVGENDQ object.
-----------------------------------------------------------------------------*/                     
#define zheng_svm_DEFAULTS { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                       (void (*)(long))zheng_svm_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in SVGEN_DQ.C
------------------------------------------------------------------------------*/
void zheng_svm_calc(zheng_svm_handle);

