
typedef struct {
	/* Need initialization */
	float		ki;			// Integral gain
	float		kp;			// Propotional gain
	float		max;		// Maximum output
	float		Ts;			// sample time
	/* Do not need initialization */
	float		ref;		// Reference value
	float		fb;			// Feedback value
	float		err;
	float		up;
	float		ui;
	float		ui_sum;
	float		out;
	void 	 (*calc)();
} PI_REG;

#define PI_REG_DEFAULTS {0, \
                          0, \
                          0, \
                          0, \
                          0,       \
						  0,	   \
						  0,	   \
						  0,	   \
						  0,	   \
						  0,	   \
						  0,	   \
						  (void (*)(long))PI_calc}

typedef PI_REG *r;

void PI_calc(PI_REG *r)
{

	r->err = r->ref - r->fb;
	r->ui = r->ki * r->err * r->Ts;
	r->ui_sum += r->ui;
	if (r->ui_sum > r->max)
		r->ui_sum -=  r->ui;
	else if (r->ui_sum < -r->max)
		r->ui_sum -=  r->ui;


	r->up = r->kp * r->err;
	r->out = r->up + r->ui_sum;

	if (r->out > r->max)
	{
		r->out = r->max;
	//	r->ui -= r->ki * r->err;
	}
	else if (r->out < -r->max)
	{
		r->out = -r->max;
	//	r->ui -= r->ki * r->err;
	}
}
