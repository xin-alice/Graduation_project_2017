

typedef struct { float  rmp_freq; 		/* Input: Ramp frequency (pu) */	
		 	     float  step_angle;	/* Parameter: Maximum step angle (pu) */
		 	     float  Ts;		
	 	 	     float  angle_rg;			/* Variable: Step angle (pu) */					  
			     float  rmp_gain;			/* Input: Ramp gain (pu) */
			     float  angle_out;  	 	/* Output: Ramp signal (pu) */	
			     float  rmp_offset;		/* Input: Ramp offset (pu) */
			     float  base_volt;
			     float  volt_out;				 
	  	  	     void  (*calc)();	  	/* Pointer to calculation function */ 
			   } RAMPGEN;	            

typedef RAMPGEN *RAMPGEN_handle;  
/*------------------------------------------------------------------------------
      Object Initializers
------------------------------------------------------------------------------*/                       
#define RAMPGEN_DEFAULTS {0,0,0,0,1,0,1,0,0, \
                         (void (*)(long))rampgen_calc }

/*------------------------------------------------------------------------------
      Funtion prototypes
------------------------------------------------------------------------------*/                                               
void rampgen_calc(RAMPGEN_handle);


