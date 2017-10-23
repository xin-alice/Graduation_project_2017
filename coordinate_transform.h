/* =================================================================================
File name:       PARK.H (IQ version)                    
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the PARK.

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 05-15-2002		Release	Rev 1.0                                                   
------------------------------------------------------------------------------*/

typedef struct {  float  xalpha;  		/* Input: stationary d-axis stator variable */
				  float  xbeta;		    /* Input: stationary q-axis stator variable */
				  float  ang;			/* Input: rotating angle (pu) */
				  float  xd;			/* Output: rotating d-axis stator variable */
				  float  xq;			/* Output: rotating q-axis stator variable */
				  float  sin_ang;
				  float  cos_ang;
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } PARK;	            

typedef PARK *PARK_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/                     
#define PARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
			 			 0, \
                          0, \
              			  (void (*)(long))park_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in PARK.C
------------------------------------------------------------------------------*/
void park_calc(PARK_handle);
//===================================================================================================//

/******************************************************************************/
typedef struct {  float  xalpha;  	/* Output: stationary d-axis stator variable */
				  float  xbeta;		/* Output: stationary q-axis stator variable */
				  float  ang;		/* Input: rotating angle (pu) */
				  float  xd;		/* Input: rotating d-axis stator variable */
				  float  xq;		/* Input: rotating q-axis stator variable */
				  float  sin_ang;
				  float  cos_ang;
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } IPARK;	            

typedef IPARK *IPARK_handle;	            


/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/                     
#define IPARK_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
                          0, \
              			  (void (*)(long))ipark_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in IPARK.C
------------------------------------------------------------------------------*/
void ipark_calc(IPARK_handle);

//===================================================================================================//
typedef struct {  float  as;  		/* Output: phase-a stator variable  */
				  float  bs;			/* Output: phase-b stator variable  */
				  float  cs;			/* Output: phase-c stator variable  */
				  float  xalpha;			/* Input: stationary d-axis stator variable  */
				  float  xbeta;			/* Input: stationary q-axis stator variable  */
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } CLARKE;	            

typedef CLARKE *CLARKE_handle;
//===================================================================================================//
/*-----------------------------------------------------------------------------
Default initalizer for the ICLARKE object.
-----------------------------------------------------------------------------*/                     
#define CLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0,\
              			  (void (*)(long))clarke_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in ICLARKE.C
------------------------------------------------------------------------------*/
void clarke_calc(CLARKE_handle);


//===================================================================================================//

/******************************************************************************/

typedef struct {  float  as;  		/* Output: phase-a stator variable  */
				  float  bs;			/* Output: phase-b stator variable  */
				  float  cs;			/* Output: phase-c stator variable  */
				  float  xalpha;			/* Input: stationary d-axis stator variable  */
				  float  xbeta;			/* Input: stationary q-axis stator variable  */
		 	 	  void  (*calc)();	/* Pointer to calculation function */ 
				 } ICLARKE;	            

typedef ICLARKE *ICLARKE_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the ICLARKE object.
-----------------------------------------------------------------------------*/                     
#define ICLARKE_DEFAULTS { 0, \
                          0, \
                          0, \
                          0, \
                          0,\
              			  (void (*)(long))iclarke_calc }

/*------------------------------------------------------------------------------
Prototypes for the functions in ICLARKE.C
------------------------------------------------------------------------------*/
void iclarke_calc(ICLARKE_handle);


//===================================================================================================//
