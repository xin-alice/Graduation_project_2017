
#include "pi_funciton.h"

void PiStep(PI_REG *r)
{
	float temp;

	r->err = r->ref - r->fb;
	temp = r->err_sum + r->err;
	r->out =r->ki* temp + r->kp* r->err;

	if (r->out > r->max)
		r->out = r->max;
	else if (r->out < - r->max)
		r->out = - r->max;
	else
		r->err_sum = temp;
}
