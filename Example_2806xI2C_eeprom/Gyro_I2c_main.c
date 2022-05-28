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
void I2CA_Init(void);
void pass(void);
void fail(void);

// Macros
#define Gyro_READ_ATTEMPTS        10
#define Gyro_DATA_BYTES           32
#define wait(x)                  DELAY_US(100000*x);
#define wait_0_2                 DELAY_US(200000);
#define wait_0_04                DELAY_US(40000);
#define wait_0_015               DELAY_US(15000);


// Globals
struct I2CHandle Gyro;
Uint16 ControlAddr[1];
Uint16 TxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 RxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 Status = 0x0000;
Uint16 ControlBuffer[1];
Uint16 k=0;
Uint8 Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
Uint8 Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
float64 gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float64 aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float64 ax, ay, az, gx, gy, gz, mx, my, mz;



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
    ControlAddr[0]         = WHO_AM_I_MPU9250;
    Gyro.SlaveAddr         = 0x68;
    Gyro.NumOfControlBytes = 1;
    Gyro.NumOfDataBytes    = 1;
    Gyro.pMsgBuffer        = RxMsgBuffer;
    Gyro.pControlBuffer    = ControlAddr;
    Gyro.Timeout           = 100;
    Status = I2C_MasterRead(&Gyro);

    ReadBytes(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, RxMsgBuffer, &Gyro);

    //
    // Reset function
    //
    TxMsgBuffer[0] = 0x80;
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(0.1);
    // End of reset function

    //
    // Calibrate function
    //
    Uint16 ii, packet_count, fifo_count;
    int32 gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    Uint8 data[12];

    TxMsgBuffer[0] = 0x80;
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(0.1);

    TxMsgBuffer[0] = 0x01;
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);

    TxMsgBuffer[0] = 0x00;
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, TxMsgBuffer, &Gyro);
    wait(0.2);

    TxMsgBuffer[0] = 0x00;

    Status = WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x0C;
    Status = WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    wait(0.015);


    TxMsgBuffer[0] = 0x01;
    Status = WriteByte(MPU9250_ADDRESS, CONFIG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    Status = WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, TxMsgBuffer, &Gyro);
    Status = WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, TxMsgBuffer, &Gyro);

    Uint16  gyrosensitivity  = 131;
    Uint16  accelsensitivity = 16384;

    TxMsgBuffer[0] = 0x40;
    Status = WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x78;
    Status = WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    wait(0.04);

    TxMsgBuffer[0] = 0x00;
    Status = WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = 0x00;
    Status = ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, RxMsgBuffer, &Gyro);

    fifo_count = ((Uint16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];
    packet_count = fifo_count/12;

    for (ii = 0; ii < packet_count; ii++)
    {
        int16 accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        Status = ReadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, RxMsgBuffer, &Gyro); // read data for averaging
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


    gyroBias[0] = (float64) gyro_bias[0]/(float64) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyroBias[1] = (float64) gyro_bias[1]/(float64) gyrosensitivity;
    gyroBias[2] = (float64) gyro_bias[2]/(float64) gyrosensitivity;

    int32 accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    RxMsgBuffer[0] = 0x00;
    RxMsgBuffer[1] = 0x00;
    Status = ReadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, RxMsgBuffer, &Gyro); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16) ((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];

    Status = ReadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, RxMsgBuffer, &Gyro);
    accel_bias_reg[1] = (int16) ((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];

    Status = ReadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, RxMsgBuffer, &Gyro);
    accel_bias_reg[2] = (int16) ((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];

    Uint32 mask = 1; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    Uint8 mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++)
    {
        if(accel_bias_reg[ii] & mask)
            {
                mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
            }
    }
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);


    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Output scaled accelerometer biases for manual subtraction in the main program
    accelBias[0] = (float64)accel_bias[0]/(float64)accelsensitivity;
    accelBias[1] = (float64)accel_bias[1]/(float64)accelsensitivity;
    accelBias[2] = (float64)accel_bias[2]/(float64)accelsensitivity;


    // End of calibrate function
    wait(0.2);
    //
    // Init Function
    //
    TxMsgBuffer[0] = 0x00;
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(0.1);
    TxMsgBuffer[0] = 0x81;
    Status = WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x03;
    Status = WriteByte(MPU9250_ADDRESS, CONFIG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x04;
    Status = WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, TxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = 0x00;
    Status = ReadBytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x02;
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x18;
    RxMsgBuffer[0] = RxMsgBuffer[0] | Gscale << 3;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    Status = WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, TxMsgBuffer, &Gyro);

    Status = ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x18;
    RxMsgBuffer[0] = RxMsgBuffer[0] | Ascale << 3;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    Status = WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, TxMsgBuffer, &Gyro);

    Status = ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x0F;
    RxMsgBuffer[0] = RxMsgBuffer[0] | 0x03;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    Status = WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, TxMsgBuffer, &Gyro);

    TxMsgBuffer[0] = 0x22;
    Status = WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x01;
    Status = WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    wait(0.2);

    //
    // End of Init function
    //

    //
    // getGres()
    //
    switch (Gscale)
    {
      // Possible gyro scales (and their register bit settings) are:
      // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
          // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
      case GFS_250DPS:
            gRes = 250.0/32768.0;
            break;
      case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
      case GFS_1000DPS:
            gRes = 1000.0/32768.0;
            break;
      case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
    }
    // End of getGres()
    while(1)
        {
        int16 Gyro_raw[3] = {0,0,0};
        // If intPin goes high, all data registers have new data
        ReadBytes(MPU9250_ADDRESS, INT_STATUS, 1, RxMsgBuffer, &Gyro);
        if( RxMsgBuffer[0] & 0x0001)
          {  // On interrupt, check if data ready interrupt

            ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, RxMsgBuffer, &Gyro);  // Read the x/y/z adc values
            // Now we'll calculate the accleration value into actual g's
            Gyro_raw[0] = (int16)(((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
            Gyro_raw[1] = (int16)(((int16)RxMsgBuffer[2] << 8) | RxMsgBuffer[3]) ;
            Gyro_raw[2] = (int16)(((int16)RxMsgBuffer[4] << 8) | RxMsgBuffer[5]) ;
            gx = (float64)Gyro_raw[0]*gRes - gyroBias[0];  // get actual g value, this depends on scale being set
            gy = (float64)Gyro_raw[1]*gRes - gyroBias[1];
            gz = (float64)Gyro_raw[2]*gRes - gyroBias[2];
          }

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
