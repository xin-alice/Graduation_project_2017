
#include "IQmathLib.h"         /* Include header for IQmath library */
/* Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file */
#include "filter.h"

void lpf_calc(LPF *v)
{	// y += tc*wc*(x-y_1)
	v->y += _IQmpy(v->Tc,_IQmpy(v->wc,(v->x - v->y_old)));
	v->y_old = v->y;
	v->lyh_scope1 = v->x - v->y_old;
	v->lyh_scope2 = v->x;
}

void lpfi_calc(LPFI *v)
{
	_iq	lpfi_tmp1;
	_iq	lpfi_tmp2;
	//v->y =v->y + _IQmpy(v->Tc,(v->x - _IQmpy(v->wc,v->y_old)));
	lpfi_tmp1 = _IQmpy(v->wc,v->y_old);
	lpfi_tmp2 = v->x - lpfi_tmp1;
	lpfi_tmp1 = _IQmpy(v->Tc,lpfi_tmp2);

	v->y = v->y + lpfi_tmp1;

	v->y_old = v->y;
}
