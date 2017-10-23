

typedef struct { float target_value; 	/* Input: Target input (pu) */
				 float  setpt_value;		/* Output: Target output (pu) */				 
				 long  rmp_dly_max;		/* Parameter: Maximum delay rate (Q0) - independently with global Q */			
				 long  rmp_delay_cntl;  /* Variable: Incremental delay (Q0) - independently with global Q */	
		 	 	 float  rmp_lo_limit;		/* Parameter: Minimum limit (pu) */					  
				 float  rmp_hi_limit;		/* Parameter: Maximum limit (pu) */
				 float  step_freq;
				 long  s_eq_t_flg;		/* Output: Flag output (Q0) - independently with global Q */
		  	  	 void  (*calc)();	  	/* Po_iqer to calculation function */ 
				 } RMPCNTL;	            

typedef RMPCNTL *RMPCNTL_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/                     
#define RMPCNTL_DEFAULTS { 0, \
							0, \
                         	100, \
                          	0, \
                           100, \
                          	-100, \
							0, \
                          	1, \
                   			(void (*)(long))rmp_cntl_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in RMP_CNTL.C
------------------------------------------------------------------------------*/
void rmp_cntl_calc(RMPCNTL_handle);


