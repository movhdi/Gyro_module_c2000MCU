/*
 * I2C_MPU9250.h
 *
 *  Created on: Mar 28, 2022
 *      Author: mmova
 */

#ifndef I2C_MPU9250_H_
#define I2C_MPU9250_H_

//
// I2C Message Structure
//
struct I2CMSG_Gyro {
    //
    // Word stating what state msg is in:
    //   I2C_MSGCMD_INACTIVE = do not send msg
    //   I2C_MSGCMD_BUSY = msg start has been sent,
    //                     awaiting stop
    //   I2C_MSGCMD_SEND_WITHSTOP = command to send
    //       master trans msg complete with a stop bit
    //   I2C_MSGCMD_SEND_NOSTOP = command to send
    //       master trans msg without the stop bit
    //   I2C_MSGCMD_RESTART = command to send a restart
    //       as a master receiver with a stop bit
    //
    Uint16 MsgStatus;

    //
    // I2C address of slave msg is intended for
    //
    Uint16 SlaveAddress;

    //
    // Register address of MPU9250
    //
    Uint16 RegisterAdress;

    //
    // Num of valid bytes in (or to be put in MsgBuffer)
    //
    Uint16 NumOfBytes;



    //
    // Array holding msg data - max that MAX_BUFFER_SIZE can be is 4 due to
    // the FIFO's
    //
    Uint16 MsgBuffer[I2C_MAX_BUFFER_SIZE];

    //
    // Define a bit to know if the transmitting should end with a STP or not
    char STP_bit;
};




#endif /* I2C_MPU9250_H_ */
