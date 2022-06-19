/*
 * F2806x_EPWM.c
 *
 *  Created on: Jun 18, 2022
 *      Author: mmova
 */

#include "F2806x_Device.h"
#include "F2806x_Examples.h"

void InitEPwm(void)
{


}


void InitEPwmGpio(void)
{
    #ifdef DSP28_EPWM3
    InitEPwm3Gpio();
    #endif
}


#if DSP28_EPWM3

void InitEPwm3Gpio(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
    EDIS;
}
#endif
