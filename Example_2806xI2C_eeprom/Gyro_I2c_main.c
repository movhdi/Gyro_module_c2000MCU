/*
 * Gyro_I2c_main.c
 *
 *  Created on: Apr 14, 2022
 *      Author: mmova
 */



#include "Gyro_module_MPU9250.h"
#include "main.h"


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
    Uint8 i = 0;

    // Fill TX and RX buffer arrays
    for(i=0;i<MAX_BUFFER_SIZE;i++)
    {
        TxMsgBuffer[i] = i;
        RxMsgBuffer[i] = 0;
    }

    Reset_MPU9250();
    Init_MPU9250();
    wait(2);
    Init_AK8963();
    getGres();
    getAres();
    getMres();


    //magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    RxMsgBuffer[0] = 0;
    RxMsgBuffer[1] = 0;

    ReadBytes(AK8963_ADDRESS, AK8963_WHO_AM_I, 1, RxMsgBuffer, &Gyro);
    WhoAmI_AK8963 = RxMsgBuffer[0];
    while(1)
        {
        int16 Raw_Data[3] = {0,0,0};
        // If intPin goes high, all data registers have new data
        ReadBytes(MPU9250_ADDRESS, INT_STATUS, 1, RxMsgBuffer, &Gyro);
        if( RxMsgBuffer[0] & 0x0001)
          {  // On interrupt, check if data ready interrupt

            ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, RxMsgBuffer, &Gyro);  // Read the x/y/z adc values
            // Now we'll calculate the accleration value into actual g's
            Raw_Data[0] = (int16)(((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            Raw_Data[1] = (int16)(((int16)RxMsgBuffer[2] << 8) | RxMsgBuffer[3]) ;
            Raw_Data[2] = (int16)(((int16)RxMsgBuffer[4] << 8) | RxMsgBuffer[5]) ;
            gx = (float32)Raw_Data[0]*gRes - gyroBias[0];  // get actual g value, this depends on scale being set
            gy = (float32)Raw_Data[1]*gRes - gyroBias[1];
            gz = (float32)Raw_Data[2]*gRes - gyroBias[2];

            ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, RxMsgBuffer, &Gyro);
            // Now we'll calculate the accleration value into actual g's
            Raw_Data[0] = (int16)(((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            Raw_Data[1] = (int16)(((int16)RxMsgBuffer[2] << 8) | RxMsgBuffer[3]) ;
            Raw_Data[2] = (int16)(((int16)RxMsgBuffer[4] << 8) | RxMsgBuffer[5]) ;
            ax = (float32)Raw_Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            ay = (float32)Raw_Data[1]*aRes - accelBias[1];
            az = (float32)Raw_Data[2]*aRes - accelBias[2];


            ReadBytes(AK8963_ADDRESS, AK8963_ST1, 1, RxMsgBuffer, &Gyro);
            if( RxMsgBuffer[0] & 0x01)
            {
                ReadBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, RxMsgBuffer, &Gyro);
                Uint8 c = RxMsgBuffer[6];
                if(!(c & 0x08))
                { // Check if magnetic sensor overflow set, if not then report data
                    Raw_Data[0] = (int16)(((int16)RxMsgBuffer[1] << 8) | RxMsgBuffer[0]);  // Turn the MSB and LSB into a signed 16-bit value
                    Raw_Data[1] = (int16)(((int16)RxMsgBuffer[3] << 8) | RxMsgBuffer[2]);  // Data stored as little Endian
                    Raw_Data[2] = (int16)(((int16)RxMsgBuffer[5] << 8) | RxMsgBuffer[4]);
                    // Calculate the magnetometer values in milliGauss
                    // Include factory calibration per data sheet and user environmental corrections
                    mx = (float32)Raw_Data[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
                    my = (float32)Raw_Data[1]*mRes*magCalibration[1] - magbias[1];
                    mz = (float32)Raw_Data[2]*mRes*magCalibration[2] - magbias[2];
                }
            }


          }

        deltat = (float32)EPwm3Regs.TBCTR;
        deltat = deltat * ((float32)(1.0f/703125.0f));
        EPwm3Regs.TBCTR = 0x0000;
        MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

            yaw_temp   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
            pitch_temp = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll_temp  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch = pitch_temp *  180.0f / PI;
            yaw   = yaw_temp * 180.0f / PI;
            //yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            roll  = roll_temp * 180.0f / PI;

            ReadBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, RxMsgBuffer, &Gyro);
            temperature = (int16)(((int16)RxMsgBuffer[0]) << 8 | RxMsgBuffer[1]) ;  // Turn the MSB and LSB into a 16-bit value
            temp = ((float32) temperature) / 333.87f + 21.0f;
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

    //
    // Interrupt where we will change the Compare Values
    //
//    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
//    EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
//    EPwm3Regs.ETPS.bit.INTPRD = ET_DISABLE;           // Generate INT on 3rd event

    //
    // Start by increasing the compare A and decreasing compare B
    //
//    epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
//    epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    //
    // Start the cout at 0
    //
//    epwm3_info.EPwmTimerIntCount = 0;
//    epwm3_info.EPwmRegHandle = &EPwm3Regs;
//    epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;
//    epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
//    epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
//    epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
}
