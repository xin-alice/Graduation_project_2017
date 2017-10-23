#include "common_struct.h"
#include <math.h>
//====================================================================================================//
typedef struct
{	float uab;
	float ubc;
	float uca;
	float usa;
	float usb;
	float usc;
	float isa;
	float isb;
	float isc;
	float udc;

	float ws;
	float Ls;
	float udc_ref;
	float iq_ref;

	float ualpha;
	float ubeta;
	float theta;
	float ud;
	float uq;
	float ialpha;
	float ibeta;
	float id;
	float iq;

	float id_ref;
	float ud_ref;
	float uq_ref;
	float ualpha_ref;
	float ubeta_ref;
	float ua_ref;
	float ub_ref;
	float uc_ref;

	PI_REG PI_udc;
	PI_REG PI_id;
	PI_REG PI_iq;

	void  (*init)();
	void  (*calc)();

} VC_Grid;

//typedef VC_Grid *VCG_handle;

#define VC_Grid_DEFAULTS {0, \
						  0, \
						  0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
                          0, \
                          0, \
						  0, \
						  0, \
						  0, \
                          0, \
                          0, \
						  0, \
						  PI_REG_DEFAULTS, \
						  PI_REG_DEFAULTS, \
						  PI_REG_DEFAULTS, \
						  (void (*)(long))VC_Grid_Init,   \
						  (void (*)(long))VC_Grid_Calc}

//void VC_Grid_Init(VCG_handle);
//void VC_Grid_Calc(VCG_handle);

//==================================================================================================//
#pragma CODE_SECTION(atan2, "ramfuncs")
#pragma CODE_SECTION(cos, "ramfuncs")
#pragma CODE_SECTION(sin, "ramfuncs")
#pragma CODE_SECTION(VC_Grid_Init, "ramfuncs")
void VC_Grid_Init(VC_Grid *p)
{
//	p->udc_ref = 600;
//	p->iq_ref = 0;

	p->PI_udc.kp = 0.5;
	p->PI_udc.ki = 0.01;
	p->PI_udc.max = 10;

	p->PI_id.kp = 5;//10;
	p->PI_id.ki = 0.2;
	p->PI_id.max = 200;
	p->PI_id.ui = -50;

	p->PI_iq.kp = 5;//10;
	p->PI_iq.ki = 0.2;
	p->PI_iq.max = 200;
	p->PI_iq.ui = -50;

	p->theta = 0;
	p->ws = 50 * DPI;
	p->Ls = 0.005;

}

#pragma CODE_SECTION(VC_Grid_Calc, "ramfuncs")
void VC_Grid_Calc(VC_Grid *p)
{
	CLARKE  VolClarke = CLARKE_DEFAULTS;
	PARK VolPark = PARK_DEFAULTS;
	IPARK VolIPark = IPARK_DEFAULTS;
	ICLARKE  VolIClarke = ICLARKE_DEFAULTS;

	CLARKE  CurClarke = CLARKE_DEFAULTS;
	PARK CurPark = PARK_DEFAULTS;

	float cos_theta=0;
	float sin_theta=0;

	p->usa = (2* p->uab + p->ubc)/3;
	p->usb = (p->ubc - p->uab)/3;
	p->usc = -p->usa - p->usb;

	VolClarke.as = p->usa;
	VolClarke.bs = p->usb;
	VolClarke.cs = p->usc;
	VolClarke.calc(&VolClarke);
	p->theta = atan2(VolClarke.xbeta, VolClarke.xalpha);

	if (p->theta<0)
		p->theta+=DPI;
	else if (p->theta>DPI)
		p->theta-=DPI;

	cos_theta = cos(p->theta);
	sin_theta = sin(p->theta);

	VolPark.xalpha = VolClarke.xalpha;
	VolPark.xbeta = VolClarke.xbeta;
	VolPark.cos_ang= cos_theta;
	VolPark.sin_ang= sin_theta;
	VolPark.calc(&VolPark);
	p->ud = VolPark.xd;
	p->uq = VolPark.xq;

	CurClarke.as = p->isa;
	CurClarke.bs = p->isb;
	CurClarke.cs = p->isc;
	CurClarke.calc(&CurClarke);
	CurPark.xalpha = CurClarke.xalpha;
	CurPark.xbeta = CurClarke.xbeta;
	CurPark.cos_ang= cos_theta;
	CurPark.sin_ang= sin_theta;
	CurPark.calc(&CurPark);
	p->id = CurPark.xd;
	p->iq = CurPark.xq;
	
	p->PI_udc.ref = p->udc_ref;//rc1.setpt_value;
	p->PI_udc.fb = p->udc;
	p->PI_udc.calc(&(p->PI_udc));
	p->id_ref = p->PI_udc.out;

	p->PI_id.ref = p->id_ref;
	p->PI_id.fb = p->id;
	p->PI_id.calc(&(p->PI_id));

	p->PI_iq.ref = p->iq_ref;
	p->PI_iq.fb = p->iq;
	p->PI_iq.calc(&(p->PI_iq));
	
	p->ud_ref = p->ud - p->PI_id.out + p->iq * p->Ls * p->ws;
	p->uq_ref = p->uq - p->PI_iq.out - p->id * p->Ls * p->ws;

	if (p->ud_ref>=350)
		p->ud_ref = 350;
	if (p->uq_ref>=350)
		p->uq_ref = 350;

	VolIPark.xd = p->ud_ref;
	VolIPark.xq = p->uq_ref;
	VolIPark.cos_ang = cos_theta;
	VolIPark.sin_ang = sin_theta;
	VolIPark.calc(&VolIPark);
	VolIClarke.xalpha = VolIPark.xalpha;
	VolIClarke.xbeta = VolIPark.xbeta;
	VolIClarke.calc(&VolIClarke);
	p->ua_ref = VolIClarke.as;
	p->ub_ref = VolIClarke.bs;
	p->uc_ref = VolIClarke.cs;
}




