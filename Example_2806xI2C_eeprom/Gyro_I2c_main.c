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
void InitEPwm3Example(void);
void pass(void);
void fail(void);

// Macros
#define Gyro_READ_ATTEMPTS        10
#define Gyro_DATA_BYTES           32
#define wait(x)                  DELAY_US(100000*x);
#define wait_0_2                 DELAY_US(200000);
#define wait_0_04                DELAY_US(40000);
#define wait_0_015               DELAY_US(15000);

#define EPWM3_TIMER_TBPRD  2000  // Period register
#define EPWM3_MAX_CMPA      950
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB     1050
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0


//typedefs

typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
}EPWM_INFO;

// Globals
struct I2CHandle Gyro;
EPWM_INFO epwm3_info;
Uint16 ControlAddr[1];
Uint16 TxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 RxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 Status = 0x0000;
Uint16 ControlBuffer[1];
Uint16 k=0;
Uint8 Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
Uint8 Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
float32 gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float32 aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float32 ax, ay, az, gx, gy, gz, mx, my, mz;
Uint8 Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
Uint8 Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
float32 magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}; // Factory mag calibration and mag bias


float32 q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float32 deltat = 0.0f;
int32 delt_t = 0; // used to control display output rate
int32 count = 0;  // used to control display output rate
float32 pitch, yaw, roll;
Uint64 cnt =0;

void MadgwickQuaternionUpdate(float32 ax, float32 ay, float32 az, float32 gx, float32 gy, float32 gz, float32 mx, float32 my, float32 mz);

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


    gyroBias[0] = (float32) gyro_bias[0]/(float32) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyroBias[1] = (float32) gyro_bias[1]/(float32) gyrosensitivity;
    gyroBias[2] = (float32) gyro_bias[2]/(float32) gyrosensitivity;

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
    accelBias[0] = (float32)accel_bias[0]/(float32)accelsensitivity;
    accelBias[1] = (float32)accel_bias[1]/(float32)accelsensitivity;
    accelBias[2] = (float32)accel_bias[2]/(float32)accelsensitivity;


    // End of calibrate function
    wait(2);
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
    // initAK8963(magCalibration)
    //
    // First extract the factory calibration for each magnetometer axis
    TxMsgBuffer[0] = 0x00;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro);// Power down magnetometer
    wait(0.01);

    TxMsgBuffer[0] = 0x0F;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro); // Enter Fuse ROM access mode
    wait(0.01);
    ReadBytes(AK8963_ADDRESS, AK8963_ASAX, 3, RxMsgBuffer, &Gyro);// Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float32)(RxMsgBuffer[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] =  (float32)(RxMsgBuffer[1] - 128)/256.0f + 1.0f;
    magCalibration[2] =  (float32)(RxMsgBuffer[2] - 128)/256.0f + 1.0f;

    TxMsgBuffer[0] = 0x00;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro); // Power down magnetometer
    wait(0.01);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    TxMsgBuffer[0] = Mscale << 4 | Mmode;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro);// Set magnetometer data resolution and sample ODR
    wait(0.01);
    // End of initAK8963(magCalibration)

    //
    // getGres()
    //
    switch (Gscale)
    {
      // Possible gyro scales (and their register bit settings) are:
      // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
          // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
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

    //
    // getAres()
    //
    switch (Ascale)
    {
      // Possible accelerometer scales (and their register bit settings) are:
      // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
          // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
      case AFS_2G:
            aRes = 2.0/32768.0;
            break;
      case AFS_4G:
            aRes = 4.0/32768.0;
            break;
      case AFS_8G:
            aRes = 8.0/32768.0;
            break;
      case AFS_16G:
            aRes = 16.0/32768.0;
            break;
    }
    // End of getAres()


    //
    // getMres()
    //

    switch (Mscale)
    {
      // Possible magnetometer scales (and their register bit settings) are:
      // 14 bit resolution (0) and 16 bit resolution (1)
      case MFS_14BITS:
            mRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
            break;
      case MFS_16BITS:
            mRes = 10.0*4912.0/32760.0; // Proper scale to return milliGauss
            break;
    }
    //
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
            if( RxMsgBuffer[0] & 0x0001)
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
        deltat = deltat * ((float32)(1.0f/7500000.0f));
        EPwm3Regs.TBCTR = 0x0000;
        MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

            //var1 = 2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
            //var2 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
            //var1 = atan((double)var1);
            //var2 = -asinl(var2);
            yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
            pitch = -asinl(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch *= 180.0f / PI;
            yaw   *= 180.0f / PI;
            //yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            roll  *= 180.0f / PI;
            cnt = 0;

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
    EPwm3Regs.TBCTL.bit.CLKDIV = 0b110;

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
    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

    //
    // Start by increasing the compare A and decreasing compare B
    //
    epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP;
    epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN;

    //
    // Start the cout at 0
    //
    epwm3_info.EPwmTimerIntCount = 0;
    epwm3_info.EPwmRegHandle = &EPwm3Regs;
    epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA;
    epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
    epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
    epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
}




void MadgwickQuaternionUpdate(float32 ax, float32 ay, float32 az, float32 gx,
                              float32 gy, float32 gz, float32 mx, float32 my, float32 mz)
{
    float32 q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        float32 norm;
        float32 hx, hy, _2bx, _2bz;
        float32 s1, s2, s3, s4;
        float32 qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        float32 _2q1mx;
        float32 _2q1my;
        float32 _2q1mz;
        float32 _2q2mx;
        float32 _4bx;
        float32 _4bz;
        float32 _2q1 = 2.0f * q1;
        float32 _2q2 = 2.0f * q2;
        float32 _2q3 = 2.0f * q3;
        float32 _2q4 = 2.0f * q4;
        float32 _2q1q3 = 2.0f * q1 * q3;
        float32 _2q3q4 = 2.0f * q3 * q4;
        float32 q1q1 = q1 * q1;
        float32 q1q2 = q1 * q2;
        float32 q1q3 = q1 * q3;
        float32 q1q4 = q1 * q4;
        float32 q2q2 = q2 * q2;
        float32 q2q3 = q2 * q3;
        float32 q2q4 = q2 * q4;
        float32 q3q3 = q3 * q3;
        float32 q3q4 = q3 * q4;
        float32 q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = sqrt(abs(ax * ax) + abs(ay * ay) + abs(az * az));
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(abs(mx * mx) + abs(my * my) + abs(mz * mz));
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f/norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1 ;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f/norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

}
