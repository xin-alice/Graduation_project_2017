

typedef struct {  	float	x;				/* Input:Input of the low pass filter (PU) */
					float	Tc;				/* Parameter:Sampling period (PU) */
					float	wc;				/* Parameter:Cut off frequency for low pass filter (PU) */
					float	y_old;			/* Variable:Output of the last cycle (PU) */
					float	y;				/* Output:Output of the low pass filter (PU) */
					void  (*calc)();	/* Po_iqer to calculation function */ 
				 } LPF;
                  
#define LPF_DEFAULTS {	  0,    \
	                      0,    \
	                      0,    \
	                      0,    \
	                      0,    \
	                      (void (*)(long))lpf_calc }
/*------------------------------------------------------------------------------*/
typedef struct {  	float	x;				/* Input:Input of the low pass filter (PU) */
					float	Tc;				/* Parameter:Sampling period (PU) */
					float 	wc;				/* Parameter:Cut off frequency for low pass filter (PU) */
					float	y_old;			/* Variable:Output of the last cycle (PU) */
					float	y;				/* Output:Output of the low pass filter (PU) */
					void  (*calc)();	/* Po_iqer to calculation function */ 
				 } LPFI;	            
                     
#define LPFI_DEFAULTS {0,       \
	                      0,    \
	                      0,    \
	                      0,    \
	                      0,    \
	 		              (void (*)(long))lpfi_calc }
/*------------------------------------------------------------------------------*/

void lpf_calc(LPF *v)
{	// y += tc*wc*(x-y_1)
	v->y += v->Tc * v->wc* (v->x - v->y_old);
	v->y_old = v->y;

}

void lpfi_calc(LPFI *v)
{
	v->y += v->Tc * (v->x - v->wc * v->y_old);

	v->y_old = v->y;
}

