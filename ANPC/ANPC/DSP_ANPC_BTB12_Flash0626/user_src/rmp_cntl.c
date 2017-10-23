

#include "rmp_cntl.h"

void rmp_cntl_calc(RMPCNTL *v)
{	

	
      if (((v->target_value - v->setpt_value)> v->step_freq)||((v->target_value - v->setpt_value)<(-v->step_freq) ))
      {
         v->rmp_delay_cntl += 1;
         if (v->rmp_delay_cntl >= v->rmp_dly_max)
         {
           v->rmp_delay_cntl = 0;
           if (v->target_value > v->setpt_value)
           {
            v->setpt_value += v->step_freq; 
           }
           else
           {
            v->setpt_value -= v->step_freq;  
         
           }
         }        
      }
      else  v->s_eq_t_flg = 0x7FFFFFFF; 
//---------------------------------------------------------------------------------//
	 if (v->setpt_value > v->rmp_hi_limit)
        v->setpt_value = v->rmp_hi_limit; 
     if (v->setpt_value < v->rmp_lo_limit)
        v->setpt_value = v->rmp_lo_limit;
//---------------------------------------------------------------------------------//
/*
	 if (v->setpt_value > v->target_value)
        v->setpt_value = v->target_value; 
     if (v->setpt_value < (-v->target_value) )
        v->setpt_value = -v->target_value;	
*/
//---------------------------------------------------------------------------------//     
}


