/*
 * Gyro_Module_MPU9250.c
 *
 *  Created on: Mar 26, 2022
 *      Author: mmova
 */
// Includes

#include "Gyro_Module_MPU9250.h"

// Globals
struct I2CHandle Gyro;

Uint8 Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
Uint8 Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
Uint8 Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
Uint8 Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution

int16 temperature = 0;

Uint16 Status = 0x0000;
Uint16 ControlAddr[1];
Uint16 TxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 RxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 ControlBuffer[1];
Uint16 WhoAmI_AK8963 = 0;

int32 delt_t = 0; // used to control display output rate

float32 gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float32 aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float32 ax, ay, az, gx, gy, gz, mx, my, mz;
float32 magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}, magScale[3] = {0,0,0}; // Factory mag calibration and mag bias
float32 q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float32 deltat = 0.0f;
float32 pitch=0, yaw=0, roll=0,pitch_temp=0,yaw_temp=0,roll_temp=0;
float32 temp = 0;


void Reset_MPU9250(void){
    TxMsgBuffer[0] = 0x80;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(100);
}


//
// initAK8963(magCalibration) // double tick
//
void Init_AK8963(void)
{
    // First extract the factory calibration for each magnetometer axis
    TxMsgBuffer[0] = 0x00;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro);// Power down magnetometer
    wait(100);

    TxMsgBuffer[0] = 0x0F;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro); // Enter Fuse ROM access mode
    wait(100);

    ReadBytes(AK8963_ADDRESS, AK8963_ASAX, 3, RxMsgBuffer, &Gyro);// Read the x-, y-, and z-axis calibration values
    magCalibration[0] =  (float32)(RxMsgBuffer[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
    magCalibration[1] =  (float32)(RxMsgBuffer[1] - 128)/256.0f + 1.0f;
    magCalibration[2] =  (float32)(RxMsgBuffer[2] - 128)/256.0f + 1.0f;

    TxMsgBuffer[0] = 0x00;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro); // Power down magnetometer
    wait(100);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    TxMsgBuffer[0] = Mscale << 4 | Mmode;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, TxMsgBuffer, &Gyro); // Set magnetometer data resolution and sample ODR
    wait(10);
}
// End of initAK8963(magCalibration)


//
// Calibrate function // tick tick
//
void MPU_9250_Calibrate(void)
{
    Uint16 ii, packet_count, fifo_count;
    int32 gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    Uint8 data[12];

    TxMsgBuffer[0] = 0x80;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(100);

    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);

    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, TxMsgBuffer, &Gyro);
    wait(200);

    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x0C;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    wait(15);


    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, CONFIG, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, TxMsgBuffer, &Gyro);
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, TxMsgBuffer, &Gyro);

    Uint16  gyrosensitivity  = 131; // = 131 LSB/degrees/sec
    Uint16  accelsensitivity = 16384; // = 16384 LSB/g

    TxMsgBuffer[0] = 0x40;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = 0x78;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    wait(40);

    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, TxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = 0x00;
    ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, RxMsgBuffer, &Gyro);

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
    if(accel_bias[2] > 0L)
    {accel_bias[2] -= (int32) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else
    {accel_bias[2] += (int32) accelsensitivity;}
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

    //Push gyro biases to hardware registers
    TxMsgBuffer[0] = data[0];
    WriteByte(MPU9250_ADDRESS, XG_OFFSET_H, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[1];
    WriteByte(MPU9250_ADDRESS, XG_OFFSET_L, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[2];
    WriteByte(MPU9250_ADDRESS, YG_OFFSET_H, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[3];
    WriteByte(MPU9250_ADDRESS, YG_OFFSET_L, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[4];
    WriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[5];
    WriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, TxMsgBuffer, &Gyro);

    gyroBias[0] = (float32) gyro_bias[0]/(float32) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    gyroBias[1] = (float32) gyro_bias[1]/(float32) gyrosensitivity;
    gyroBias[2] = (float32) gyro_bias[2]/(float32) gyrosensitivity;

    int32 accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    RxMsgBuffer[0] = 0x00;
    RxMsgBuffer[1] = 0x00;
    ReadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, RxMsgBuffer, &Gyro); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32) ((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];

    ReadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, RxMsgBuffer, &Gyro);
    accel_bias_reg[1] = (int32) ((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];

    ReadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, RxMsgBuffer, &Gyro);
    accel_bias_reg[2] = (int32) ((int16)RxMsgBuffer[0] << 8) | RxMsgBuffer[1];

    Uint32 mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
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

    //Push accelerometer biases to hardware registers
    TxMsgBuffer[0] = data[0];
    WriteByte(MPU9250_ADDRESS, XA_OFFSET_H, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[1];
    WriteByte(MPU9250_ADDRESS, XA_OFFSET_L, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[2];
    WriteByte(MPU9250_ADDRESS, YA_OFFSET_H, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[3];
    WriteByte(MPU9250_ADDRESS, YA_OFFSET_L, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[4];
    WriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, TxMsgBuffer, &Gyro);
    TxMsgBuffer[0] = data[5];
    WriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, TxMsgBuffer, &Gyro);


    // Output scaled accelerometer biases for manual subtraction in the main program
    accelBias[0] = (float32)accel_bias[0]/(float32)accelsensitivity;
    accelBias[1] = (float32)accel_bias[1]/(float32)accelsensitivity;
    accelBias[2] = (float32)accel_bias[2]/(float32)accelsensitivity;

}
// End of calibrate function

//
// calibrateMag function
//
void calibrateMag(float32 * dest1, float32 * dest2)
{
    Uint16 ii = 0, jj = 0, sample_count = 0;
    int32 mag_bias[3] = {0,0,0}, mag_scale[3] = {0,0,0};
    int16 mag_max[3] = {-32767,-32767,-32767}, mag_min[3] = {32767,32767,32767},mag_temp[3] = {0,0,0};
    if(Mmode == 0x02)
    {
        sample_count = 128;
    }
    if(Mmode == 0x06)
    {
        sample_count = 1500;
    }

    for(ii = 0; ii < sample_count ; ii++)
    {
        readMagData(mag_temp);
        for(jj = 0; jj < 3 ; jj++)
        {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        if(Mmode == 0x02)
            {
                wait(135);  // at 8 Hz ODR, new mag data is available every 125 ms
            }
        if(Mmode == 0x06)
            {
                wait(12);  // at 100 Hz ODR, new mag data is available every 10 ms
            }
    }

    // get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float32) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float32) mag_bias[1]*mRes*magCalibration[1];
    dest1[2] = (float32) mag_bias[2]*mRes*magCalibration[2];

    // get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float32 avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float32)mag_scale[0]);
    dest2[1] = avg_rad/((float32)mag_scale[1]);
    dest2[2] = avg_rad/((float32)mag_scale[2]);

}

// end of calibrateMag()

//
//  readMagData()
//
void readMagData(int16 * destination)
{
    ReadBytes(AK8963_ADDRESS, AK8963_ST1, 1, RxMsgBuffer, &Gyro);
    if( RxMsgBuffer[0] & 0x01)
    {
        ReadBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, RxMsgBuffer, &Gyro);
        Uint8 c = RxMsgBuffer[6];
        if(!(c & 0x08))
        { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = (int16)(((int16)RxMsgBuffer[1] << 8) | RxMsgBuffer[0]);  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = (int16)(((int16)RxMsgBuffer[3] << 8) | RxMsgBuffer[2]);  // Data stored as little Endian
            destination[2] = (int16)(((int16)RxMsgBuffer[5] << 8) | RxMsgBuffer[4]);
        }
    }
}
// end of reagMagData

//
// Init Function  //  double tick
//
void Init_MPU9250(void)
{
    TxMsgBuffer[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(100);

    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(100);

    TxMsgBuffer[0] = 0x03;
    WriteByte(MPU9250_ADDRESS, CONFIG, TxMsgBuffer, &Gyro);
    wait(100);

    TxMsgBuffer[0] = 0x04;
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, TxMsgBuffer, &Gyro);
    wait(100);

    RxMsgBuffer[0] = 0x00;
    ReadBytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x03;
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x18;
    RxMsgBuffer[0] = RxMsgBuffer[0] | Gscale << 3;
    wait(100);

    TxMsgBuffer[0] = RxMsgBuffer[0];
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, TxMsgBuffer, &Gyro);
    wait(100);

    ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x18;
    RxMsgBuffer[0] = RxMsgBuffer[0] | Ascale << 3;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, TxMsgBuffer, &Gyro);
    wait(100);

    ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, RxMsgBuffer, &Gyro);
    RxMsgBuffer[0] = RxMsgBuffer[0] & ~0x0F;
    RxMsgBuffer[0] = RxMsgBuffer[0] | 0x03;
    TxMsgBuffer[0] = RxMsgBuffer[0];
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, TxMsgBuffer, &Gyro);
    wait(100);

    TxMsgBuffer[0] = 0x22;
    WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, TxMsgBuffer, &Gyro);

/*
    TxMsgBuffer[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, TxMsgBuffer, &Gyro);
*/
    wait(1000);

}
//
// End of Init function
//




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


void WriteByte(Uint8 SlaveAddress, Uint16 RegAddress , Uint16 *data, struct I2CHandle *I2C_Params )
{
    I2C_Params->NumOfControlBytes = 1;
    I2C_Params->NumOfDataBytes = 1;
    I2C_Params->SlaveAddr = SlaveAddress;
    I2C_Params->pMsgBuffer = data;
    ControlBuffer[0] = RegAddress;
    I2C_Params->pControlBuffer = ControlBuffer;
    I2C_MasterWrite(I2C_Params);
}

void ReadBytes(Uint8 SlaveAddress, Uint16 RegAddress , Uint8 count,Uint16 *dest, struct I2CHandle *I2C_Params )
{
    I2C_Params->NumOfControlBytes = 1;
    I2C_Params->NumOfDataBytes = count;
    I2C_Params->SlaveAddr = SlaveAddress;
    I2C_Params->pMsgBuffer = dest;
    ControlBuffer[0] = RegAddress;
    I2C_Params->pControlBuffer = ControlBuffer; // the address is 8 bits long but the memory is 16bits
    I2C_MasterRead(I2C_Params);
}

void getAres(void)
{
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
}

void getGres(void)
{
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
}


void getMres(void)
{
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
}
