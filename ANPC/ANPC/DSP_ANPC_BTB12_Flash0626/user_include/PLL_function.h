
typedef struct 
{
	float		ref;		// Reference value
	float		fb;			// Feedback value
	float		err;
	float		err_sum;	// Integral of err
	float		out;
	void 	 (*Calc)();
} PLL;

//typedef PLL *p;

void PLL_calc(PLL *p)
{
p->out=p->ref;

}


