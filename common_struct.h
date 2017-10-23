//==================================================================================================//
typedef struct {  float  xalpha;  		/* Input: stationary d-axis stator variable */
				  float  xbeta;		    /* Input: stationary q-axis stator variable */
				  float  ang;			/* Input: rotating angle (pu) */
				  float  xd;			/* Output: rotating d-axis stator variable */
				  float  xq;			/* Output: rotating q-axis stator variable */
				  float  sin_ang;
				  float  cos_ang;
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } PARK;	            
                 
#define PARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
			 			 0, \
                          0, \
              			  (void (*)(long))park_calc }

void park_calc(PARK *PARK_handle);
//==================================================================================================//
typedef struct {  float  xalpha;  	/* Output: stationary d-axis stator variable */
				  float  xbeta;		/* Output: stationary q-axis stator variable */
				  float  ang;		/* Input: rotating angle (pu) */
				  float  xd;		/* Input: rotating d-axis stator variable */
				  float  xq;		/* Input: rotating q-axis stator variable */
				  float  sin_ang;
				  float  cos_ang;
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } IPARK;	            
	                           
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
              			  (void (*)(long))ipark_calc }

void ipark_calc(IPARK *IPARK_handle);
//===================================================================================================//
typedef struct {  float  as;  		/* Output: phase-a stator variable  */
				  float  bs;			/* Output: phase-b stator variable  */
				  float  cs;			/* Output: phase-c stator variable  */
				  float  xalpha;			/* Input: stationary d-axis stator variable  */
				  float  xbeta;			/* Input: stationary q-axis stator variable  */
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } CLARKE;	            
                    
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0,\
              			  (void (*)(long))clarke_calc }

void clarke_calc(CLARKE *CLARKE_handle);
//===================================================================================================//
typedef struct {  float  as;  		/* Output: phase-a stator variable  */
				  float  bs;			/* Output: phase-b stator variable  */
				  float  cs;			/* Output: phase-c stator variable  */
				  float  xalpha;			/* Input: stationary d-axis stator variable  */
				  float  xbeta;			/* Input: stationary q-axis stator variable  */
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } ICLARKE;	            
                
#define ICLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0,\
              			  (void (*)(long))iclarke_calc }

void iclarke_calc(ICLARKE *ICLARKE_handle);
//===================================================================================================//
typedef struct {
	/* Need initialization */
	float		ki;			// Integral gain
	float		kp;			// Propotional gain
	float		max;		// Maximum output
	/* Do not need initialization */
	float		ref;		// Reference value
	float		fb;			// Feedback value
	float		err;
	float		err_sum;	// Integral of err
	float		up;
	float		ui;
	float		out;
	void 	 (*calc)();
} PI_REG;

#define PI_REG_DEFAULTS {0, \
                          0, \
                          0, \
                          0,       \
						  0,	   \
						  0,	   \
						  0,	   \
						  0,	   \
						  0,	   \
						  0,	   \
						  (void (*)(long))PI_calc}
void PI_calc(PI_REG *PI_handle);
//====================================================================================================//
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
void lpf_calc(LPF *LPF_handle);
//======================================================================================================//
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
void lpfi_calc(LPFI *LPFI_handle);
//========================================================================================================//



