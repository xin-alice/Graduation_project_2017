//vol_balance.h

typedef struct
{	float usin;
	float ufc;
	float udc1;
	float udc2;
	float ufc_ref;
	float i;
	float duty;
	Uint16 redundant;
	Uint16 level;
	Uint16 level_last;
	Uint16 PWM_mode;
	void  (*correct)();
	void  (*level_calc)();
	void  (*FC_balancing)();
	void  (*PWM_mode_calc)();
} phase_state;


#define phase_state_DEFAULTS {0, \
							  0, \
							  0, \
	                          0, \
	                          0, \
	                          0, \
							  0, \
							  0, \
							  0, \
							  0, \
							  0x0000,  \
						  (void (*)(long))phase_vol_correct,   \
						  (void (*)(long))phase_level_calc,    \
						  (void (*)(long))FC_balancing,   	   \
						  (void (*)(long))PWM_mode_calc}

//void phase_level_calc(phase_state_handle);
void phase_vol_correct()
{


}

#pragma CODE_SECTION(phase_level_calc, "ramfuncs")
void phase_level_calc(phase_state *psh)
{
	if (psh->usin >= 4)
		{
			psh->usin = 4;
			psh->level= 3;
		}
	else if (psh->usin <= 4 && psh->usin > 3)
		{
			psh->level=3;
		}
	else if (psh->usin <=3 && psh->usin >2)
		{
			psh->level=2;
		}
//************************************************

//保证串联双管工作在基频
	else if (psh->usin ==2)
		{
			if(psh->level_last<=1)
			psh->level=1;
			else
			psh->level=2;
		}

//************************************************
	else if (psh->usin <2 && psh->usin >1)
		{
			psh->level=1;
		}
	else if (psh->usin <=1 && psh->usin >0)
		{
			psh->level=0;
		}
	else if (psh->usin <=0)
		{
			psh->usin =0;
			psh->level=0;
		}
	psh->duty = psh->usin - psh->level;
	psh->level_last = psh->level;
}

#pragma CODE_SECTION(FC_balancing, "ramfuncs")
void FC_balancing(phase_state *psh)
{
	if (((psh->i > 0)&&(psh->ufc > psh->ufc_ref))||((psh->i < 0)&&(psh->ufc < psh->ufc_ref)))
		psh->redundant = 0;
	else
		psh->redundant = 1;

}

#pragma CODE_SECTION(PWM_mode_calc, "ramfuncs")
void PWM_mode_calc(phase_state *psh)
{
	Uint16 mode1;
	Uint16 mode2;
	Uint16 mode3;

	if (psh->level==0)
	{
        if (psh->redundant==0)
		{
            mode1=1;
            mode2=1;
            mode3=3;
		}
        else
		{
            mode1=1;
            mode2=3;
            mode3=1;
        }
	}
    else if (psh->level==1)
	{
        if (psh->redundant==0)
		{
            mode1=1;
            mode2=3;
            mode3=2;
		}
        else
		{
            mode1=1;
            mode2=2;
            mode3=3;
        }
	}
    else if (psh->level==2)
	{
		if (psh->redundant==0)
		{
            mode1=2;
            mode2=1;
            mode3=3;
		}
        else
		{
            mode1=2;
            mode2=3;
            mode3=1;
        }
	}
    else if (psh->level==3)
	{
        if (psh->redundant==0)
		{
            mode1=2;
            mode2=3;
            mode3=2;
		}
        else
		{
            mode1=2;
            mode2=2;
            mode3=3;
        }
	}

	psh->PWM_mode =(3<<6)+(mode3<<4) + (mode2<<2) + mode1;
}

