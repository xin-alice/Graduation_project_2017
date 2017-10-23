
typedef struct 	{ _iq  Ualfa; 			/* Input: reference alfa-axis phase voltage   */
				  _iq  Ubeta;			/* Input: reference beta-axis phase voltage   */
				  int  sector;
				  _iq  Vdc;
				  _iq  T1;				
				  _iq  T2;		
				  _iq  T0;
				  _iq  T0_half;
				  _iq  Ta0;				/* Output: reference phase-a switching function  */			
				  _iq  Tb0;				/* Output: reference phase-b switching function   */
				  _iq  Tc0;				/* Output: reference phase-c switching function   */
				  _iq  Ta;				/* Output: reference phase-a switching function  */			
				  _iq  Tb;				/* Output: reference phase-b switching function   */
				  _iq  Tc;				/* Output: reference phase-c switching function   */
				  _iq  U0;
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

