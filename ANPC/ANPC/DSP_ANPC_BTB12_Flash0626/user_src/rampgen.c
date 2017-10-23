
//#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "personal_set.h"
#include "rampgen.h"

void rampgen_calc(RAMPGEN *v)
{	

       v->step_angle = v->rmp_freq*DPI*v->Ts; 
        v->angle_rg += v->step_angle;      
        
        if (v->angle_rg> DPI)
          v->angle_rg -= DPI;
        if (v->angle_rg< 0)
          v->angle_rg += DPI;

//       v->rmp_out = v->angle_rg*v->rmp_gain + v->rmp_offset;
	   v->angle_out = v->angle_rg + v->rmp_offset;
       if (v->angle_out>DPI)
          v->angle_out -= DPI;
        else if (v->angle_out<0)
          v->angle_out += DPI;
//-----------------------------------------------------------------------//
/*
	    v->angle_out = (v->angle_rg + v->rmp_offset)*0.15915494309;
       if (v->angle_out>1.0)
          v->angle_out -= 1.0;
        else if (v->angle_out<0)
          v->angle_out += 1.0;
*/
//-----------------------------------------------------------------------//
	v->volt_out = v->base_volt* v->rmp_freq;

//-----------------------------------------------------------------------//

}


