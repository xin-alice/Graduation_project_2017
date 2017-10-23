
#include "coordinate_transform.h"
#include <math.h>
void park_calc(PARK *v)
{	
  
 //   float cos_ang,sin_ang;

/* Using look-up IQ sine table */  
//     v->sin_ang = sin(v->ang);
//     v->cos_ang = cos(v->ang);

     v->xd = v->xalpha*v->cos_ang + v->xbeta*v->sin_ang;
     v->xq =v->xbeta*v->cos_ang - v->xalpha*v->sin_ang;

}

void ipark_calc(IPARK *v)
{	
   
//   float cos_ang,sin_ang;
   
/* Using look-up IQ sine table */  
//     sin_ang = sin(v->ang);
//     cos_ang = cos(v->ang);
     v->xalpha = v->xd*v->cos_ang - v->xq*v->sin_ang;
     v->xbeta = v->xq*v->cos_ang + v->xd*v->sin_ang;  
}
void clarke_calc(CLARKE *v)
{	

    v->xalpha = v->as;
	v->xbeta = (v->bs - v->cs)*0.57735; //
}
void iclarke_calc(ICLARKE *v)
{	

    v->as = v->xalpha*0.81649658;
	v->bs = -0.40824829*v->xalpha + 0.70710678*v->xbeta;
    v->cs = -(v->as + v->bs); 

}

