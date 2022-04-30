/*
 * Gyro_I2c_main.c
 *
 *  Created on: Apr 14, 2022
 *      Author: mmova
 */


#include "DSP28x_Project.h"
//#include "I2C_MPU9250.h"
#include "Gyro_module_MPU9250.h"
#include "i2cLib_FIFO_polling.h"

//funtion prototypes
void  I2CA_Init(void);
void  fail(void);


// Macros
#define EEPROM_READ_ATTEMPTS        10
#define EEPROM_DATA_BYTES           32


// Globals
struct I2CHandle Gyro;
Uint16 ControlAddr[1];
Uint16 TxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 RxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 Status;
Uint16 SlaveAddress_I2C = 0x68;
Uint16 ControlBuffer[1];
Uint16 k=0;
Uint16 True_address = 0x68;
extern Uint16 Gscale;
extern Uint16 Ascale;
extern float64 gyroBias[3], accelBias[3];





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
    Uint16 i;


    // Fill TX and RX buffer arrays
    for(i=0;i<MAX_BUFFER_SIZE;i++)
    {
        TxMsgBuffer[i] = i;
        RxMsgBuffer[i] = 0;
    }

    Gyro.Timeout = 100;
    Gyro.SlaveAddr = 0x68;

    Status = ReadBytes(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, RxMsgBuffer, &Gyro);
    //
    // Reset function
    //
    TxMsgBuffer[0] = 0x80;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    // End of reset function

    //
    // Calibrate function
    //
    TxMsgBuffer[0] = 0x80;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x0C;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, CONFIG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, TxMsgBuffer, &Gyro);
    Uint16  gyrosensitivity  = 131;
    Uint16  accelsensitivity = 16384;
    TxMsgBuffer[0] = 0x40;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x78;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    DELAY_US(1000);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = 0x00;
    ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, RxMsgBuffer, &Gyro);
    Uint16 ii, packet_count, fifo_count;
    int32 gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    Uint8 data[12];
    fifo_count = ((Uint16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];
    packet_count = fifo_count/12;

    for (ii = 0; ii < packet_count; ii++)
    {
        int16 accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, RxMsgBuffer, &Gyro); // read data for averaging
        accel_temp[0] = (int16) (((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16) (((int16)RxMsgBuffer[2] << 8) | RxMsgBuffer[3]  ) ;
        accel_temp[2] = (int16) (((int16)RxMsgBuffer[4] << 8) | RxMsgBuffer[5]  ) ;
        gyro_temp[0]  = (int16) (((int16)RxMsgBuffer[6] << 8) | RxMsgBuffer[7]  ) ;
        gyro_temp[1]  = (int16) (((int16)RxMsgBuffer[8] << 8) | RxMsgBuffer[9]  ) ;
        gyro_temp[2]  = (int16) (((int16)RxMsgBuffer[10] << 8) | RxMsgBuffer[11]) ;

        accel_bias[0] += (int32) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32) accel_temp[1];
        accel_bias[2] += (int32) accel_temp[2];
        gyro_bias[0]  += (int32) gyro_temp[0];
        gyro_bias[1]  += (int32) gyro_temp[1];
        gyro_bias[2]  += (int32) gyro_temp[2];
    }
    accel_bias[0] /= (int32) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32) packet_count;
    accel_bias[2] /= (int32) packet_count;
    gyro_bias[0]  /= (int32) packet_count;
    gyro_bias[1]  /= (int32) packet_count;
    gyro_bias[2]  /= (int32) packet_count;
    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32) accelsensitivity;}
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    gyro_bias[0] = (float64) gyro_bias[0]/(float64) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyro_bias[1] = (float64) gyro_bias[1]/(float64) gyrosensitivity;
    gyro_bias[2] = (float64) gyro_bias[2]/(float64) gyrosensitivity;







    // End of calibrate function

    //
    // Init Function
    //
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x81;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x03;
    WriteByte(MPU9250_ADDRESS, CONFIG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x04;
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, TxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = 0x00;
    ReadBytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x02;
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x18;
    RxMsgBuffer[0] = RxMsgBuffer[0] | Gscale << 3;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, TxMsgBuffer, &Gyro);

    ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x18;
    RxMsgBuffer[0] = RxMsgBuffer[0] | Ascale << 3;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, TxMsgBuffer, &Gyro);

    ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x0F;
    RxMsgBuffer[0] = RxMsgBuffer[0] | 0x03;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, TxMsgBuffer, &Gyro);

    TxMsgBuffer[0] = 0x22;
    WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    DELAY_US(1000);

    //
    // End of Init function
    //

    ReadBytes(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, RxMsgBuffer, &Gyro);

    while(1)
        {

        }

}



void   I2CA_Init(void)
{
    //
    // Initialize I2C
    //

    I2caRegs.I2CPSC.all = 8;        // Prescaler - need 7-12 Mhz on module clk
    I2caRegs.I2CCLKL = 10;          // NOTE: must be non zero
    I2caRegs.I2CCLKH = 5;           // NOTE: must be non zero
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
