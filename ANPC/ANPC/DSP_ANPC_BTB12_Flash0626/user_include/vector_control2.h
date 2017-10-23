//vector_control.h

typedef struct
{	float id;
	float id_ref;
	float iq;
	float iq_ref;
	float speed;
	float speed_ref;
	float torq_ref;
	float usd_ref;
	float usq_ref;
	float usdc;
	float usqc;
	float flux;
	float flux_last;
	float wr;
	float ws;
	float wsl;
	float theta_s;
	float theta_s_last;
	float cos_ang;
	float sin_ang;
	void  (*calc)();
} VectCtrl_motor;


#define VectCtrl_motor_DEFAULTS {0, \
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
						  (void (*)(long))VectCtrl_motor_calc}

void VectCtrl_motor_calc()
{


}

typedef struct
{	float id;
	float id_ref;
	float iq;
	float iq_ref;
	float speed;
	float speed_ref;
	float ud_ref;
	float uq_ref;
	float flux;
	float wr;
	float ws;
	float wsl;
	float theta_s;
	float theta_s_last;
	float cos_ang;
	float sin_ang;
	void  (*calc)();
} VectCtrl_grid;


#define VectCtrl_grid_DEFAULTS {0, \
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
						  (void (*)(long))VectCtrl_grid_calc}

void VectCtrl_grid_calc()
{


}




