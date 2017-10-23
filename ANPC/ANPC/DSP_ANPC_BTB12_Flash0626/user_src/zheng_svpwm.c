


#include "zheng_svpwm.h"
void zheng_svm_calc(zheng_svm *v)
{	
	int sector;
//	float u_alfa1;
	float u_beta1;
//	float vdc1_inv;

	float t_sum;
//	u_alfa1 = v->Ualfa*0.8660254;
//	u_beta1 = v->Ubeta*0.5;

//	vdc1_inv = 1.41421356/v->Vdc ;
	v->V_ba_inv = 1.5/v->Vdc;
	u_beta1 = v->Ubeta*0.577350269; //Ubeta/sqrt(3);
	if(v->Ubeta >0)//sector 1,2,3
	{
		if (v->Ualfa >u_beta1)
			sector = 1;
		else if(v->Ualfa < (-u_beta1))
			sector = 3;
		else 
			sector = 2;
	}
	else  //sector 4,5,6
	{
		if (v->Ualfa >(-u_beta1) )
			sector = 6;
		else if(v->Ualfa < u_beta1)
			sector = 4;
		else 
			sector = 5;
	}
	
	switch(sector)
	{
		case 1: //sector 1 
		{
			v->T1 = (v->Ualfa-u_beta1)*v->V_ba_inv;
			v->T2 = 2*u_beta1*v->V_ba_inv;
			t_sum = v->T1 + v->T2;
			if (t_sum>1.0)
			{
				v->T1 = v->T1/t_sum;
				v->T2 = v->T2/t_sum; 
			}
			v->T0 = 1.0- v->T1 - v->T2;
			v->T0_half = v->T0*0.5;
			v->Ta = v->T0_half + v->T2+ v->T1;
			v->Tb = v->T0_half + v->T2;
			v->Tc = v->T0_half; 
			break;
		}
		case 2:
		{
			v->T1 = (v->Ualfa + u_beta1)*v->V_ba_inv;
			v->T2 = (-v->Ualfa + u_beta1)*v->V_ba_inv;
			t_sum = v->T1 + v->T2;
			if (t_sum>1.0)
			{
				v->T1 = v->T1/t_sum;
				v->T2 = v->T2/t_sum; 
			}
			v->T0 = 1.0- v->T1 - v->T2;
			v->T0_half = v->T0*0.5;
			v->Ta = v->T0_half + v->T1;
			v->Tb = v->T0_half + v->T2 + v->T1;
			v->Tc = v->T0_half; 
			break;
		}
		case 3:
		{
			v->T1 = 2*u_beta1*v->V_ba_inv;
			v->T2 = (-v->Ualfa - u_beta1)*v->V_ba_inv;
			t_sum = v->T1 + v->T2;
			if (t_sum>1.0)
			{
				v->T1 = v->T1/t_sum;
				v->T2 = v->T2/t_sum; 
			}
			v->T0 = 1.0- v->T1 - v->T2;
			v->T0_half = v->T0*0.5;
			v->Ta = v->T0_half;
			v->Tb = v->T0_half + v->T2 + v->T1;
			v->Tc = v->T0_half + v->T2; 
			break;
		}
		case 4:
		{
			v->T1 = (-v->Ualfa + u_beta1)*v->V_ba_inv;
			v->T2 = -2*u_beta1*v->V_ba_inv;
			t_sum = v->T1 + v->T2;
			if (t_sum>1.0)
			{
				v->T1 = v->T1/t_sum;
				v->T2 = v->T2/t_sum; 
			}
			v->T0 = 1.0- v->T1 - v->T2;
			v->T0_half = v->T0*0.5;
			v->Ta = v->T0_half;
			v->Tb = v->T0_half + v->T1;
			v->Tc = v->T0_half + v->T2 + v->T1; 
			break;
		}
		case 5:
		{
			v->T1 = (-v->Ualfa - u_beta1)*v->V_ba_inv;
			v->T2 = (v->Ualfa - u_beta1)*v->V_ba_inv;
			t_sum = v->T1 + v->T2;
			if (t_sum>1.0)
			{
				v->T1 = v->T1/t_sum;
				v->T2 = v->T2/t_sum; 
			}
			v->T0 = 1.0- v->T1 - v->T2;
			v->T0_half = v->T0*0.5;
			v->Ta = v->T0_half + v->T2;
			v->Tb = v->T0_half;
			v->Tc = v->T0_half + v->T2 + v->T1; 
			break;
		}
		case 6:
		{
			v->T1 = -2*u_beta1*v->V_ba_inv;
			v->T2 = (v->Ualfa + u_beta1)*v->V_ba_inv;
			t_sum = v->T1 + v->T2;
			if (t_sum>1.0)
			{
				v->T1 = v->T1/t_sum;
				v->T2 = v->T2/t_sum; 
			}
			v->T0 = 1.0- v->T1 - v->T2;
			v->T0_half = v->T0*0.5;
			v->Ta = v->T0_half + v->T2 + v->T1;
			v->Tb = v->T0_half;
			v->Tc = v->T0_half + v->T1; 
			
		}			
	}
	v->sector = sector;
	v->Ta0 = 1.0 - v->Ta;
	v->Tb0 = 1.0 - v->Tb;
	v->Tc0 = 1.0 - v->Tc;
	v->Ta =  v->Ta0;
	v->Tb =  v->Tb0;
	v->Tc =  v->Tc0;
	
}
	
