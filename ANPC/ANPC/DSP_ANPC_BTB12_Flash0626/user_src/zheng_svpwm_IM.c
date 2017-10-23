

#include "zheng_svpwm_IM.h"

void zheng_svm_calc(zheng_svm *v)
{	
	int sector;
	_iq u_beta1;
	_iq vdc1;
	_iq vdc1_inv;

	_iq t_sum;

	u_beta1 = _IQdiv(v->Ubeta, _IQ(1.7320508));//Ubeta/sqrt(3)
	vdc1= _IQdiv(v->Vdc ,_IQ(1.5));//Vdc*2/3
	vdc1_inv = _IQdiv(_IQ(1.5),v->Vdc );
	if(v->Ubeta >0)//sector 1,2,3
	{
		if (v->Ualfa >u_beta1 )
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
			v->T1 = _IQmpy((v->Ualfa - u_beta1),vdc1_inv);
			v->T2 = _IQmpy(_IQmpy(u_beta1, _IQ(2.0)),vdc1_inv);
			t_sum = v->T1 + v->T2;
			if (t_sum>_IQ(1.0))
			{
				v->T1 = _IQdiv(v->T1, t_sum);
				v->T2 = _IQdiv(v->T2, t_sum); 
			}
			v->T0 = _IQ(1.0)- v->T1 - v->T2;
			v->T0_half = _IQmpy(v->T0 ,_IQ(0.5));
			v->Ta = v->T0_half + v->T2+ v->T1;
			v->Tb = v->T0_half + v->T2;
			v->Tc = v->T0_half; 
			break;
		}
		case 2:
		{
			v->T1 = _IQmpy((v->Ualfa + u_beta1),vdc1_inv);
			v->T2 = _IQmpy((-v->Ualfa + u_beta1),vdc1_inv);
			t_sum = v->T1 + v->T2;
			if (t_sum>_IQ(1.0))
			{
				v->T1 = _IQdiv(v->T1, t_sum);
				v->T2 = _IQdiv(v->T2, t_sum); 
			}
			v->T0 = _IQ(1.0)- v->T1 - v->T2;
			v->T0_half = _IQmpy(v->T0 ,_IQ(0.5));
			v->Ta = v->T0_half + v->T1;
			v->Tb = v->T0_half + v->T2 + v->T1;
			v->Tc = v->T0_half; 
			break;
		}
		case 3:
		{
			v->T1 = _IQmpy(_IQmpy(_IQ(2.0),u_beta1),vdc1_inv);
			v->T2 = _IQmpy((-v->Ualfa - u_beta1),vdc1_inv);
			t_sum = v->T1 + v->T2;
			if (t_sum>_IQ(1.0))
			{
				v->T1 = _IQdiv(v->T1, t_sum);
				v->T2 = _IQdiv(v->T2, t_sum); 
			}
			v->T0 = _IQ(1.0)- v->T1 - v->T2;
			v->T0_half = _IQmpy(v->T0 ,_IQ(0.5));
			v->Ta = v->T0_half;
			v->Tb = v->T0_half + v->T2 + v->T1;
			v->Tc = v->T0_half + v->T2; 
			break;
		}
		case 4:
		{
			v->T1 = _IQmpy((-v->Ualfa + u_beta1),vdc1_inv);
			v->T2 = _IQmpy(_IQmpy(_IQ(-2.0),u_beta1),vdc1_inv);
			t_sum = v->T1 + v->T2;
			if (t_sum>_IQ(1.0))
			{
				v->T1 = _IQdiv(v->T1, t_sum);
				v->T2 = _IQdiv(v->T2, t_sum); 
			}
			v->T0 = _IQ(1.0)- v->T1 - v->T2;
			v->T0_half = _IQmpy(v->T0 ,_IQ(0.5));
			v->Ta = v->T0_half;
			v->Tb = v->T0_half + v->T1;
			v->Tc = v->T0_half + v->T2 + v->T1; 
			break;
		}
		case 5:
		{
			v->T1 = _IQmpy((-v->Ualfa - u_beta1),vdc1_inv);
			v->T2 = _IQmpy(( v->Ualfa - u_beta1),vdc1_inv);
			t_sum = v->T1 + v->T2;
			if (t_sum>_IQ(1.0))
			{
				v->T1 = _IQdiv(v->T1, t_sum);
				v->T2 = _IQdiv(v->T2, t_sum); 
			}
			v->T0 = _IQ(1.0)- v->T1 - v->T2;
			v->T0_half = _IQmpy(v->T0 ,_IQ(0.5));
			v->Ta = v->T0_half + v->T2;
			v->Tb = v->T0_half;
			v->Tc = v->T0_half + v->T2 + v->T1; 
			break;
		}
		case 6:
		{
			v->T1 = _IQmpy(_IQmpy(_IQ(-2.0),u_beta1),vdc1_inv);
			v->T2 = _IQmpy(( v->Ualfa + u_beta1),vdc1_inv);
			t_sum = v->T1 + v->T2;
			if (t_sum>_IQ(1.0))
			{
				v->T1 = _IQdiv(v->T1, t_sum);
				v->T2 = _IQdiv(v->T2, t_sum); 
			}
			v->T0 = _IQ(1.0)- v->T1 - v->T2;
			v->T0_half = _IQmpy(v->T0 ,_IQ(0.5));
			v->Ta = v->T0_half + v->T2 + v->T1;
			v->Tb = v->T0_half;
			v->Tc = v->T0_half + v->T1; 
			
		}			
	}
	v->sector = sector;
	v->U0 = _IQmpy((v->Ta + v->Tb + v->Tc - _IQ(1.5)), _IQ(0.4714045));
	v->Ta0 = _IQ(1.0) - v->Ta;
	v->Tb0 = _IQ(1.0) - v->Tb;
	v->Tc0 = _IQ(1.0) - v->Tc;

	//-------------------------------------------------------------//
	if (v->Ta0 > _IQ(1.0))
		v->Ta0 = _IQ(1.0);
	if (v->Ta0 < _IQ(0.0))
		v->Ta0 = _IQ(0.0);

	if (v->Tb0 > _IQ(1.0))
		v->Tb0 = _IQ(1.0);
	if (v->Tb0 < _IQ(0.0))
		v->Tb0 = _IQ(0.0);

	if (v->Tc0 > _IQ(1.0))
		v->Tc0 = _IQ(1.0);
	if (v->Tc0 < _IQ(0.0))
		v->Tc0 = _IQ(0.0);

//-------------------------------------------------------------//
	if (v->Ta0 > _IQ(0.98))
		v->Ta0 = _IQ(0.98);
	if (v->Ta0 < _IQ(0.02))
		v->Ta0 = _IQ(0.02);

	if (v->Tb0 > _IQ(0.98))
		v->Tb0 = _IQ(0.98);
	if (v->Tb0 < _IQ(0.02))
		v->Tb0 = _IQ(0.02);

	if (v->Tc0 > _IQ(0.98))
		v->Tc0 = _IQ(0.98);
	if (v->Tc0 < _IQ(0.02))
		v->Tc0 = _IQ(0.02);

	
}
	
