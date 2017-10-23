// TI File $Revision: /main/4 $
// Checkin $Date: July 30, 2007   14:15:53 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#define ADC_usDELAY  5000L

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	    EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;//
		ADC_cal();
		EDIS;

	AdcRegs.ADCTRL1.bit.RESET = 1;		// Reset the ADC module
	asm(" RPT #10 || NOP");				// Must wait 12-cycles (worst-case) for ADC reset to take effect

	AdcRegs.ADCTRL3.all = 0x00E0;

	AdcRegs.ADCTRL3.all = 0x00C4;		// first power-up ref and bandgap circuits
	AdcRegs.ADCTRL3.bit.ADCBGRFDN= 0x3;
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x2;// 0010;

//	DSP28x_usDelay(7000);
//	DelayUs(7000);						// Wait 7ms before setting ADCPWDN
	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;	// Set ADCPWDN=1 to power main ADC
//	DSP28x_usDelay(5000);

	AdcRegs.ADCTRL1.all = 0x0e90;
	AdcRegs.ADCTRL1.bit.CPS = 1;
	AdcRegs.ADCTRL1.bit.ACQ_PS = 0xe;  //0xe  1110

	AdcRegs.ADCTRL2.all = 0x0900;

	AdcRegs.ADCREFSEL.bit.REF_SEL = 0x0; // Internal reference 2.048V
//	AdcRegs.ADCREFSEL.bit.REF_SEL = 0x1; // External reference 2.048V


	DSP28x_usDelay(5000);
    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_CLOCK_SPEED define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

//    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
//    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
}

//===========================================================================
// End of file.
//===========================================================================
