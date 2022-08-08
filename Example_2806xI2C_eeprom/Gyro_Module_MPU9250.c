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

Uint8 Ascale = AFS_8G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
Uint8 Gscale = GFS_1000DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
Uint8 Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
Uint8 Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution

int16 temperature = 0;
int16 accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16 gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16 magCount[3] = {0,0,0};    // Stores the 16-bit signed magnetometer sensor output

Uint16 ControlAddr[1];
Uint16 TxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 RxMsgBuffer[MAX_BUFFER_SIZE];
Uint16 ControlBuffer[1];
Uint16 WhoAmI_AK8963 = 0;



float32 gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float32 aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float32 ax, ay, az, gx, gy, gz, mx, my, mz;
float32 magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}, magScale[3] = {0,0,0}; // Factory mag calibration and mag bias
float32 SelfTest[6];
float32 q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float32 deltat = 0.0f;
float32 temp = 0.;
float32 a12, a22, a31, a32, a33;

MPU9255_t MPU9255;

Uint8 MPU_Init(void) // Ok
{
    //pre-def. vars
    Uint16 readData[1];

    //read MPU9255 WHOAMI
    ReadBytes(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, readData, &Gyro);

    if (readData[0] == 113) {

        //Start by performing self test and reporting values
        MPU9250SelfTest(SelfTest); // Ok

        //Calibrate gyro and accelerometers, load biases in bias registers
        MPU_9250_Calibrate(gyroBias,accelBias); // Ok
        wait(1000);

        //init Gyro and Accelerometer
        Init_MPU9250();

        //Read the WHO_AM_I register of the magnetometer
        ReadBytes(AK8963_ADDRESS, AK8963_ADDRESS, 1, readData, &Gyro);
        wait(1000);

        //Get magnetometer calibration from AK8963 ROM
        Init_AK8963(magCalibration);  // Ok

        calibrateMag(magbias, magScale); // Ok

        wait(1000);
        return 0;
    }
    return 1; // Loop forever if communication doesn't happen

}



void Reset_MPU9250(void){
    TxMsgBuffer[0] = 0x80;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, TxMsgBuffer, &Gyro);
    wait(100);
}

//
//  readAll()
//
void readAll(MPU9255_t *DataStruct)
{
    Uint16 Data[1];
    float32 pitch=0., yaw=0., roll=0.;
    ReadBytes(MPU9250_ADDRESS, INT_STATUS, 1, Data, &Gyro);
    if(Data[0] & 0x0001)
    {
        readAccelData(accelCount); // Ok
        getAres(); // Ok

        ax = (float32)accelCount[0]*aRes; // - accelBias[0];
        ay = (float32)accelCount[1]*aRes; // - accelBias[1];
        az = (float32)accelCount[2]*aRes; // - accelBias[2];

        DataStruct->AccelX = ax;
        DataStruct->AccelY = ay;
        DataStruct->AccelZ = az;

        readGyroData(gyroCount); // Ok
        getGres(); // Ok

        gx = (float32)gyroCount[0]*gRes;
        gy = (float32)gyroCount[1]*gRes;
        gz = (float32)gyroCount[2]*gRes;

        DataStruct->GyroX = gx;
        DataStruct->GyroY = gy;
        DataStruct->GyroZ = gz;

        readMagData(magCount);
        getMres();
        mx = (float32)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
        my = (float32)magCount[1]*mRes*magCalibration[1] - magbias[1];
        mz = (float32)magCount[2]*mRes*magCalibration[2] - magbias[2];
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];

        DataStruct->MagX = mx;
        DataStruct->MagY = my;
        DataStruct->MagZ = mz;
    }

    deltat = (float32)EPwm3Regs.TBCTR;
    deltat = deltat * ((float32)(1.0f/703125.0f));
    EPwm3Regs.TBCTR = 0x0000;
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   += 4.96f; // Declination at Tehran at estimatedly at Rafsanjan

    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
//    lin_ax = ax + a31;
//    lin_ay = ay + a32;
//    lin_az = az - a33;

    DataStruct->yaw = yaw;
    DataStruct->pitch = pitch;
    DataStruct->roll = roll;

    DataStruct->temperature = readTempData();

}

// end of function readAll()



//
// initAK8963(magCalibration) // double tick
//
void Init_AK8963(float32 * destination)
{
    Uint16 writeData[1];

    //First extract the factory calibration for each magnetometer axis
    // x/y/z gyro calibration data stored here
    Uint16 rawMagCalData[3];
    // First extract the factory calibration for each magnetometer axis
    writeData[0] = 0x00;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, writeData, &Gyro);// Power down magnetometer
    wait(100);

    writeData[0] = 0x0F;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, writeData, &Gyro); // Enter Fuse ROM access mode
    wait(100);

    rawMagCalData[0] = 0;
    rawMagCalData[1] = 0;
    rawMagCalData[2] = 0;
    ReadBytes(AK8963_ADDRESS, AK8963_ASAX, 3, rawMagCalData, &Gyro);// Read the x-, y-, and z-axis calibration values
    destination[0] =  (float32)(rawMagCalData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
    destination[1] =  (float32)(rawMagCalData[1] - 128)/256.0f + 1.0f;
    destination[2] =  (float32)(rawMagCalData[2] - 128)/256.0f + 1.0f;

    writeData[0] = 0x00;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, writeData, &Gyro); // Power down magnetometer
    wait(100);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeData[0] = Mscale << 4 | Mmode;
    WriteByte(AK8963_ADDRESS, AK8963_CNTL, writeData, &Gyro); // Set magnetometer data resolution and sample ODR
    wait(10);
}
// End of initAK8963(magCalibration)


//
// Calibrate function // tick tick
//
void MPU_9250_Calibrate(float32 * dest1, float32 * dest2)
{
    Uint16 writeData[1];
    Uint16 calibData[12];
    Uint16 ii, packet_count, fifo_count;
    int32 gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    writeData[0] = 0x80;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, writeData, &Gyro);
    wait(100);

    writeData[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, writeData, &Gyro);

    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_2, writeData, &Gyro);
    wait(200);

    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, INT_ENABLE, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, I2C_MST_CTRL, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, writeData, &Gyro);
    writeData[0] = 0x0C;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, writeData, &Gyro);
    wait(15);


    writeData[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, CONFIG, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, writeData, &Gyro);

    Uint16  gyrosensitivity  = 131; // = 131 LSB/degrees/sec
    Uint16  accelsensitivity = 16384; // = 16384 LSB/g

    writeData[0] = 0x40;
    WriteByte(MPU9250_ADDRESS, USER_CTRL, writeData, &Gyro);
    writeData[0] = 0x78;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, writeData, &Gyro);
    wait(40);

    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, FIFO_EN, writeData, &Gyro);
    calibData[0] = 0x00;
    ReadBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, calibData, &Gyro);

    fifo_count = ((Uint16)calibData[0] << 8) | calibData[1];
    packet_count = fifo_count/12;

    for (ii = 0; ii < packet_count; ii++)
    {
        int16 accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        ReadBytes(MPU9250_ADDRESS, FIFO_R_W, 12, calibData, &Gyro); // read data for averaging
        accel_temp[0] = (int16) (((int16)calibData[0] << 8) | calibData[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16) (((int16)calibData[2] << 8) | calibData[3]  ) ;
        accel_temp[2] = (int16) (((int16)calibData[4] << 8) | calibData[5]  ) ;
        gyro_temp[0]  = (int16) (((int16)calibData[6] << 8) | calibData[7]  ) ;
        gyro_temp[1]  = (int16) (((int16)calibData[8] << 8) | calibData[9]  ) ;
        gyro_temp[2]  = (int16) (((int16)calibData[10] << 8) | calibData[11]) ;

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
    {
        accel_bias[2] -= (int32) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32) accelsensitivity;
    }
    calibData[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    calibData[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    calibData[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    calibData[3] = (-gyro_bias[1]/4)       & 0xFF;
    calibData[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    calibData[5] = (-gyro_bias[2]/4)       & 0xFF;

    //Push gyro biases to hardware registers
    writeData[0] = calibData[0];
    WriteByte(MPU9250_ADDRESS, XG_OFFSET_H, writeData, &Gyro);
    writeData[0] = calibData[1];
    WriteByte(MPU9250_ADDRESS, XG_OFFSET_L, writeData, &Gyro);
    writeData[0] = calibData[2];
    WriteByte(MPU9250_ADDRESS, YG_OFFSET_H, writeData, &Gyro);
    writeData[0] = calibData[3];
    WriteByte(MPU9250_ADDRESS, YG_OFFSET_L, writeData, &Gyro);
    writeData[0] = calibData[4];
    WriteByte(MPU9250_ADDRESS, ZG_OFFSET_H, writeData, &Gyro);
    writeData[0] = calibData[5];
    WriteByte(MPU9250_ADDRESS, ZG_OFFSET_L, writeData, &Gyro);

    dest1[0] = (float32) gyro_bias[0]/(float32) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float32) gyro_bias[1]/(float32) gyrosensitivity;
    dest1[2] = (float32) gyro_bias[2]/(float32) gyrosensitivity;

    int32 accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    calibData[0] = 0x00;
    calibData[1] = 0x00;
    ReadBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, calibData, &Gyro); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32) ((int16)calibData[0] << 8) | calibData[1];

    ReadBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, calibData, &Gyro);
    accel_bias_reg[1] = (int32) ((int16)calibData[0] << 8) | calibData[1];

    ReadBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, calibData, &Gyro);
    accel_bias_reg[2] = (int32) ((int16)calibData[0] << 8) | calibData[1];

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


    calibData[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    calibData[1] = (accel_bias_reg[0])      & 0xFF;
    calibData[1] = calibData[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    calibData[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    calibData[3] = (accel_bias_reg[1])      & 0xFF;
    calibData[3] = calibData[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    calibData[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    calibData[5] = (accel_bias_reg[2])      & 0xFF;
    calibData[5] = calibData[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    //Push accelerometer biases to hardware registers
    writeData[0] = calibData[0];
    WriteByte(MPU9250_ADDRESS, XA_OFFSET_H, writeData, &Gyro);
    writeData[0] = calibData[1];
    WriteByte(MPU9250_ADDRESS, XA_OFFSET_L, writeData, &Gyro);
    writeData[0] = calibData[2];
    WriteByte(MPU9250_ADDRESS, YA_OFFSET_H, writeData, &Gyro);
    writeData[0] = calibData[3];
    WriteByte(MPU9250_ADDRESS, YA_OFFSET_L, writeData, &Gyro);
    writeData[0] = calibData[4];
    WriteByte(MPU9250_ADDRESS, ZA_OFFSET_H, writeData, &Gyro);
    writeData[0] = calibData[5];
    WriteByte(MPU9250_ADDRESS, ZA_OFFSET_L, writeData, &Gyro);


    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float32)accel_bias[0]/(float32)accelsensitivity;
    dest2[1] = (float32)accel_bias[1]/(float32)accelsensitivity;
    dest2[2] = (float32)accel_bias[2]/(float32)accelsensitivity;

}
// End of calibrate function

//
// self test function MPU9250SelfTest()
//
void MPU9250SelfTest(float32 * destination)
{
    Uint16 writeData[1];
    Uint16 rawTestData[6] = {0, 0, 0, 0, 0, 0};
    Uint16 selfTest[6];
    int32 gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
    float32 factoryTrim[6];
    Uint8 FS = 0;
    int ii =0;
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, writeData, &Gyro);

    writeData[0] = 0x02;
    WriteByte(MPU9250_ADDRESS, CONFIG, writeData, &Gyro);

    writeData[0] = FS<<3;
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, writeData, &Gyro);

    writeData[0] = 0x02;
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, writeData, &Gyro);

    writeData[0] = FS<<3;
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, writeData, &Gyro);

    //get average current values of gyro and acclerometer
    for(  ii = 0; ii < 200; ii++) {

        ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawTestData, &Gyro);
        aAvg[0] += (int16)(((int16)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16)(((int16)rawTestData[2] << 8) | rawTestData[3]) ;
        aAvg[2] += (int16)(((int16)rawTestData[4] << 8) | rawTestData[5]) ;

        ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawTestData, &Gyro);
        gAvg[0] += (int16)(((int16)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16)(((int16)rawTestData[2] << 8) | rawTestData[3]) ;
        gAvg[2] += (int16)(((int16)rawTestData[4] << 8) | rawTestData[5]) ;
    }

    //Get average of 200 values and store as average current readings
    for (  ii =0; ii < 3; ii++) {
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    //Configure the accelerometer for self-test
    writeData[0] = 0xE0;
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, writeData, &Gyro);
    writeData[0] = 0xE0;
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, writeData, &Gyro);
    wait(25);  // Delay a while to let the device stabilize

    //get average self-test values of gyro and acclerometer
    for(  ii = 0; ii < 200; ii++) {

        ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawTestData, &Gyro);
        aSTAvg[0] += (int16)(((int16)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16)(((int16)rawTestData[2] << 8) | rawTestData[3]) ;
        aSTAvg[2] += (int16)(((int16)rawTestData[4] << 8) | rawTestData[5]) ;

        ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawTestData, &Gyro);
        gSTAvg[0] += (int16)(((int16)rawTestData[0] << 8) | rawTestData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16)(((int16)rawTestData[2] << 8) | rawTestData[3]) ;
        gSTAvg[2] += (int16)(((int16)rawTestData[4] << 8) | rawTestData[5]) ;
    }

    //Get average of 200 values and store as average self-test readings
    for (  ii =0; ii < 3; ii++) {
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    //Configure the gyro and accelerometer for normal operation
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, writeData, &Gyro);
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, writeData, &Gyro);
    wait(25);  // Delay a while to let the device stabilize

    //Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    ReadBytes(MPU9250_ADDRESS, SELF_TEST_X_ACCEL, 1,  &selfTest[0], &Gyro);
    ReadBytes(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1,  &selfTest[1], &Gyro);
    ReadBytes(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL, 1,  &selfTest[2], &Gyro);
    ReadBytes(MPU9250_ADDRESS, SELF_TEST_X_GYRO, 1,  &selfTest[3], &Gyro);
    ReadBytes(MPU9250_ADDRESS, SELF_TEST_Y_GYRO, 1,  &selfTest[4], &Gyro);
    ReadBytes(MPU9250_ADDRESS, SELF_TEST_Z_GYRO, 1,  &selfTest[5], &Gyro);

    //Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float32)(2620/1<<FS)*(pow( 1.01 , ((float32)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float32)(2620/1<<FS)*(pow( 1.01 , ((float32)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float32)(2620/1<<FS)*(pow( 1.01 , ((float32)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
    factoryTrim[3] = (float32)(2620/1<<FS)*(pow( 1.01 , ((float32)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float32)(2620/1<<FS)*(pow( 1.01 , ((float32)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float32)(2620/1<<FS)*(pow( 1.01 , ((float32)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

    Uint32 testResults[6];

    //Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    //To get percent, must multiply by 100
    int i = 0;
    for (  i = 0; i < 3; i++) {
        testResults[i]   = 100.0*((float32)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
        testResults[i+3] = 100.0*((float32)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
    }


   for ( i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float32)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float32)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }

}

// end of self test function

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
            if(mag_temp[jj] > mag_max[jj])
                {
                    mag_max[jj] = mag_temp[jj];
                }
            if(mag_temp[jj] < mag_min[jj])
                {
                    mag_min[jj] = mag_temp[jj];
                }
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
// readAccelData()
//

void readAccelData(int16 * destination)
{
  Uint16 rawAccelData[6];  // x/y/z accel register data stored here
  //HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &rawAccelData[0], 6, i2c_timeout); // Read the six raw data registers into data array
  ReadBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawAccelData, &Gyro);
  destination[0] = ((int16)rawAccelData[0] << 8) | rawAccelData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16)rawAccelData[2] << 8) | rawAccelData[3];
  destination[2] = ((int16)rawAccelData[4] << 8) | rawAccelData[5];

}

// end of readAccelData()

//
// readGyroData
//

void readGyroData( int16 * destination)
{
  Uint16 rawGyroData[6];  // x/y/z gyro register data stored here
  //HAL_I2C_Mem_Read(I2Cx, MPU9250_ADDRESS, GYRO_XOUT_H, 1, &rawGyroData[0], 6, i2c_timeout);  // Read the six raw data registers sequentially into data array
  ReadBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawGyroData, &Gyro);
  destination[0] = ((int16)rawGyroData[0] << 8) | rawGyroData[1];  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16)rawGyroData[2] << 8) | rawGyroData[3];
  destination[2] = ((int16)rawGyroData[4] << 8) | rawGyroData[5];

}

//
//  readMagData()
//
void readMagData(int16 * destination)
{
    Uint16 readData[1];
    ReadBytes(AK8963_ADDRESS, AK8963_ST1, 1, readData, &Gyro);
    if( (readData[0] & 0x0001) == 0x0001)
    {
        Uint16 rawMagData[7];
        ReadBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, rawMagData, &Gyro);
        Uint16 c = rawMagData[6];
        if(!(c & 0x0008))
        { // Check if magnetic sensor overflow set, if not then report data
            destination[0] = (((int16)rawMagData[1] << 8) | rawMagData[0]);  // Turn the MSB and LSB into a signed 16-bit value
            destination[1] = (((int16)rawMagData[3] << 8) | rawMagData[2]);  // Data stored as little Endian
            destination[2] = (((int16)rawMagData[5] << 8) | rawMagData[4]);
        }
    }
}
// end of reagMagData

float32 readTempData(void)
{
    Uint16 tempRawData[2];
    int16 temperature ;
    ReadBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, tempRawData, &Gyro);
    temperature = (((int16)tempRawData[0]) << 8 | tempRawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
    temperature = ((float32) temperature) / 333.87f + 21.0f;
    return temperature;

}

//
// Init Function  //  double tick
//
void Init_MPU9250(void)
{
    Uint16 readData[6];
    Uint16 writeData[1];
    writeData[0] = 0x00;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, writeData, &Gyro);
    wait(100);

    writeData[0] = 0x01;
    WriteByte(MPU9250_ADDRESS, PWR_MGMT_1, writeData, &Gyro);
    wait(100);

    writeData[0] = 0x03;
    WriteByte(MPU9250_ADDRESS, CONFIG, writeData, &Gyro);
    wait(100);

    writeData[0] = 0x04;
    WriteByte(MPU9250_ADDRESS, SMPLRT_DIV, writeData, &Gyro);
    wait(100);

    readData[0] = 0x00;
    ReadBytes(MPU9250_ADDRESS, GYRO_CONFIG, 1, readData, &Gyro);
    readData[0] = readData[0] & ~0x03;
    readData[0] = readData[0] & ~0x18;
    readData[0] = readData[0] | Gscale << 3;
    wait(100);

    writeData[0] = readData[0];
    WriteByte(MPU9250_ADDRESS, GYRO_CONFIG, writeData, &Gyro);
    wait(100);

    ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG, 1, readData, &Gyro);
    readData[0] = readData[0] & ~0x18;
    readData[0] = readData[0] | Ascale << 3;

    writeData[0] = readData[0];
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG, writeData, &Gyro);
    wait(100);

    ReadBytes(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, readData, &Gyro);
    readData[0] = readData[0] & ~0x0F;
    readData[0] = readData[0] | 0x03;

    writeData[0] = readData[0];
    WriteByte(MPU9250_ADDRESS, ACCEL_CONFIG2, TxMsgBuffer, &Gyro);
    wait(100);

    writeData[0] = 0x22;
    WriteByte(MPU9250_ADDRESS, INT_PIN_CFG, writeData, &Gyro);

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
        norm = sqrtf(abs(ax * ax) + abs(ay * ay) + abs(az * az));
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f/norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrtf(abs(mx * mx) + abs(my * my) + abs(mz * mz));
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
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
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
        norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
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
        // Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
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
