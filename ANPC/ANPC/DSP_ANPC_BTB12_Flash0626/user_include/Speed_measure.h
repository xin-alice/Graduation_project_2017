				  
typedef struct {  unsigned int dir_QEP;
				  unsigned long  posi_cnt_old;  	
				  unsigned long  posi_cnt;
				  unsigned int qep_spd_cnt;
				  unsigned int encoder_linear;
				  unsigned int  Fs_cpu_M;
				  //unsigned long  delta_cnt;		   
				  int  posi_indx;// the flag of the index signal
				  unsigned int  first_index;
				  int cycle_cnt;		
				  float   posi_meca;
				  float   posi_meca_old;
				  float   posi_meca_ab;//绝对位置
				  float   posi_meca_ab_old;//绝对位置
				  float   delta_posi;		
				  float   w_meca;
				  float   w_mec_old;
				  float cap_speed;
				  float Ts;
				  float Ts_speed;
				  float encoder_linear_inv;
				  unsigned int cap_cnt;
				  unsigned int speed_meas_cnt;
				  unsigned int index_flt_cnt;
				  void  (*Init)();
//		 	 	  void  (*Update)();
				 } eQEP;	            

typedef eQEP *eQEP_handle;
//-----------------------------------------------------------------------------------------------//
#define eQEP_DEFAULTS {  0, 0,0,0,0,0,0,0,0,0,\
						0,0,0,0,0,0,0,0,0,0,  \
						0,0,0,  \
						(void (*)(long))eQEP_init, \
              			  /*(void (*)(long))eQEP_calc*/ }
//void eQEP_calc(eQEP_handle);
void eQEP_init(eQEP_handle);


