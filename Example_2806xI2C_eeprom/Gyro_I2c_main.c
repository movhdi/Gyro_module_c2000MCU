/*
 * Gyro_I2c_main.c
 *
 *  Created on: Apr 14, 2022
 *      Author: mmova
 */



#include "Gyro_module_MPU9250.h"
#include "main.h"

float32 yaw_result, pitch_result, roll_result;
//typedefs

void main(void)
{
    InitSysCtrl();
    InitI2CGpio();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    I2CA_Init();
    InitEPwm3Example();

    while(MPU_Init() == 1);

    wait(3000);

    while(1)
    {
        readAll(&MPU9255);
        yaw_result = MPU9255.yaw;
        pitch_result = MPU9255.pitch;
        roll_result = MPU9255.roll;
    }

}



void   I2CA_Init(void)
{
    //
    // Initialize I2C
    //

    I2caRegs.I2CPSC.all = 8;        // Prescaler - need 7-12 Mhz on module clk
    I2caRegs.I2CCLKL = 8;          // NOTE: must be non zero
    I2caRegs.I2CCLKH = 7;           // NOTE: must be non zero
    I2caRegs.I2CIER.all = 0x00;     // Disable all interrupts

    //
    // Take I2C out of reset. Stop I2C when suspended
    //
    I2caRegs.I2CMDR.all = 0x0020;

    return;
}

void fail(void)
{
    __asm("   ESTOP0");
    for(;;);
}


void
InitEPwm3Example(void)
{
    //
    // Setup TBCLK
    //
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    EPwm3Regs.TBPRD = 0xFFFF;       // Set timer period
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = 0b111;

    //
    // Setup shadow register load on ZERO
    //
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    //
    // Set Compare values
    //
    EPwm3Regs.CMPA.half.CMPA = EPWM3_MIN_CMPA; // Set compare A value
    EPwm3Regs.CMPB = EPWM3_MAX_CMPB;           // Set Compare B value

    //
    // Set Actions
    //
    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;    // Set PWM3A on event B, up count
    EPwm3Regs.AQCTLA.bit.CBU = AQ_CLEAR;  // Clear PWM3A on event B, up count

    EPwm3Regs.AQCTLB.bit.ZRO = AQ_TOGGLE; // Toggle EPWM3B on Zero
}
