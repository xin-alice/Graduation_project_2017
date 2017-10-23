//zero_sequence.h

#include "math.h"

float NP_cur_calc(float usin, float io, int redundant);
float Min3(float a1, float a2, float a3);
float DutyH(float x);

typedef struct
{	float uin[3];			/*三相给定参考值*/
	float uout[3];			/*叠加零序之后三相参考值*/
	float io[3];
	float umin0;
	float umid0;
	float umax0;
	int order[3];
	float uzmin;
	float uzmax;
	float uz[13];
	float umin[13];
	float umid[13];
	float umax[13];
	float Inmin[13];
	float Inmid[13];
	float Inmax[13];
	float Inp[13];
	float Inp_ref;
	float diff_Inp[13];
	int num_uz;
	int best_uz;
	int redundant[3];
	void  (*order_volt)();
	void  (*uz_calc)();
} ZERO_SEQUENCE;


#define ZERO_SEQUENCE_DEFAULTS {  {0,0,0}, \
		                          {0,0,0}, \
								  {0,0,0},  \
								  0, \
								  0, \
								  0, \
								  {0,1,2},  \
								  0, \
								  0, \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  0, \
								  {0,0,0,0,0,0,0,0,0,0,0,0,0},  \
								  1, \
								  0, \
								  {0,0,0},  \
								  (void (*)(long))order_volt,  \
							   	  (void (*)(long))uz_volt_calc}

#pragma CODE_SECTION(order_volt, "ramfuncs")
void order_volt(ZERO_SEQUENCE *zsv)
{
	if (zsv->uin[0] < zsv->uin[1])
	{
		if (zsv->uin[0] < zsv->uin[2])
		{
			if (zsv->uin[1] < zsv->uin[2])
			{
				zsv->umin0=zsv->uin[0];
				zsv->umid0=zsv->uin[1];
				zsv->umax0=zsv->uin[2];
				zsv->order[0]=0;
				zsv->order[1]=1;
				zsv->order[2]=2;
			}
			else
			{
				zsv->umin0=zsv->uin[0];
				zsv->umid0=zsv->uin[2];
				zsv->umax0=zsv->uin[1];
				zsv->order[0]=0;
				zsv->order[1]=2;
				zsv->order[2]=1;
			}
		}
		else
		{
			zsv->umin0=zsv->uin[2];
			zsv->umid0=zsv->uin[0];
			zsv->umax0=zsv->uin[1];
			zsv->order[0]=2;
			zsv->order[1]=0;
			zsv->order[2]=1;
		}
	}
	else
	{
		if (zsv->uin[0] > zsv->uin[2])
		{
			if (zsv->uin[1] > zsv->uin[2])
			{
				zsv->umin0=zsv->uin[2];
				zsv->umid0=zsv->uin[1];
				zsv->umax0=zsv->uin[0];
				zsv->order[0]=2;
				zsv->order[1]=1;
				zsv->order[2]=0;
			}
			else
			{
				zsv->umin0=zsv->uin[1];
				zsv->umid0=zsv->uin[2];
				zsv->umax0=zsv->uin[0];
				zsv->order[0]=1;
				zsv->order[1]=2;
				zsv->order[2]=0;
			}
		}
		else
		{
			zsv->umin0=zsv->uin[1];
			zsv->umid0=zsv->uin[0];
			zsv->umax0=zsv->uin[2];
			zsv->order[0]=1;
			zsv->order[1]=0;
			zsv->order[2]=2;
		}
	}

}

#pragma CODE_SECTION(uz_volt_calc, "ramfuncs")
void uz_volt_calc(ZERO_SEQUENCE *zsv)
{
	int i=0;

	zsv->uzmin = -2-zsv->umin0;
	zsv->uzmax = 2-zsv->umax0;

//************************************************

//保证串联双管工作在基频
	if (zsv->umid0<0)
	{
		if(-zsv->umid0 < zsv->uzmax)
			zsv->uzmax = -zsv->umid0;
		if (-zsv->umax0 > zsv->uzmin)
			zsv->uzmin = -zsv->umax0;
	}
	else
	{
		if(-zsv->umid0 > zsv->uzmin)
			zsv->uzmin = -zsv->umid0;
		if (-zsv->umin0 < zsv->uzmax)
			zsv->uzmax = -zsv->umin0;
	}

//**************************************************/

	zsv->uz[0] = zsv->uzmin;
	zsv->umin[0] = zsv->umin0 + zsv->uz[0];
	zsv->umid[0] = zsv->umid0 + zsv->uz[0];
	zsv->umax[0] = zsv->umax0 + zsv->uz[0];

	zsv->Inmin[0] = NP_cur_calc(zsv->umin[0],zsv->io[zsv->order[0]],zsv->redundant[zsv->order[0]]);
	zsv->Inmid[0] = NP_cur_calc(zsv->umid[0],zsv->io[zsv->order[1]],zsv->redundant[zsv->order[1]]);
	zsv->Inmax[0] = NP_cur_calc(zsv->umax[0],zsv->io[zsv->order[2]],zsv->redundant[zsv->order[2]]);
	zsv->Inp[0] = zsv->Inmin[0]+zsv->Inmid[0]+zsv->Inmax[0];
	zsv->diff_Inp[0] = fabs(zsv->Inp[0] - zsv->Inp_ref);
	for(i=1; i<=12; i++)
	{
		zsv->uz[i]=Min3(DutyH(zsv->umin[i-1]),DutyH(zsv->umid[i-1]),DutyH(zsv->umax[i-1]));


		zsv->umin[i]=zsv->umin[i-1]+zsv->uz[i];
		zsv->umid[i]=zsv->umid[i-1]+zsv->uz[i];
		zsv->umax[i]=zsv->umax[i-1]+zsv->uz[i];

		if (zsv->umax[i]>zsv->uzmax+zsv->umax0)
		{
			zsv->num_uz = i;
			break;
		}

		zsv->Inmin[i] = NP_cur_calc(zsv->umin[i],zsv->io[zsv->order[0]],zsv->redundant[zsv->order[0]]);
		zsv->Inmid[i] = NP_cur_calc(zsv->umid[i],zsv->io[zsv->order[1]],zsv->redundant[zsv->order[1]]);
		zsv->Inmax[i] = NP_cur_calc(zsv->umax[i],zsv->io[zsv->order[2]],zsv->redundant[zsv->order[2]]);
		zsv->Inp[i] = zsv->Inmin[i]+zsv->Inmid[i]+zsv->Inmax[i];
		zsv->diff_Inp[i] = fabs(zsv->Inp[i] - zsv->Inp_ref);
	}

	zsv->best_uz=0;

	for(i=0;i<=zsv->num_uz-1;i++)
	{
		if(zsv->diff_Inp[i] < zsv->diff_Inp[zsv->best_uz])
			zsv->best_uz = i;
	}

/*
//--------------------------------------------
	zsv->best_uz=-1;
	for(i=0;i<=zsv->num_uz-1;i++)
	{
		if ((floor(zsv->umax[i])+floor(zsv->umid[i])+floor(zsv->umin[i])>=-4)&&(ceil(zsv->umax[i])+ceil(zsv->umid[i])+ceil(zsv->umin[i])<=4))
        {
            if (zsv->best_uz<0)
                zsv->best_uz=i;
			else 
			{
				if(zsv->diff_Inp[i] < zsv->diff_Inp[zsv->best_uz])
					zsv->best_uz = i;
			}
		}
	}
	
	if (zsv->best_uz<0)
		zsv->best_uz=0;
//--------------------------------------------*/

	zsv->uout[zsv->order[0]] = zsv->umin[zsv->best_uz];
	zsv->uout[zsv->order[1]] = zsv->umid[zsv->best_uz];
	zsv->uout[zsv->order[2]] = zsv->umax[zsv->best_uz];

}

#pragma CODE_SECTION(NP_cur_calc, "ramfuncs")
float NP_cur_calc(float usin, float io, int redundant)
{
	float Inp=0;

	if ((usin>=-2)&(usin<-1))
		Inp = (2+usin)*redundant*io;
	else if ((usin>=-1)&(usin<0))
	{
		if (redundant==0)
			Inp = (1+usin)*io;
		else
			Inp = io;
	}
	else if ((usin>=0)&(usin<1))
	{
		if (redundant==1)
			Inp = (1-usin)*io;
		else
			Inp = io;
	}
	else if ((usin>=1)&(usin<=2))
	{
		Inp = (2-usin)*(1-redundant)*io;
	}
	else
		Inp = 0;

	return Inp;

}

#pragma CODE_SECTION(Min3, "ramfuncs")
float Min3(float a1, float a2, float a3)
{
	float a_min;
	if (a1<=a2)
	{
		if (a2<=a3)
			a_min = a1;
		else
		{
			if (a1<=a3)
				a_min = a1;
			else
				a_min = a3;
		}
	}
	else
	{
		if (a2>=a3)
			a_min = a3;
		else
			a_min = a2;
	}

	return a_min;
}

#pragma CODE_SECTION(DutyH, "ramfuncs")
float DutyH(float x)
{
	float y;
	if(x>=0)
	{
		y = (int)(x) +1-x;

	}
	else
	{
		y = (int)(x)-x;
		if (y<=0)
			y=y+1;

	}
	return y;

}


