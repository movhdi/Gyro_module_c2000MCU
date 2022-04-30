
#ifndef I2CLIB_FIFO_POLLING_H_
#define I2CLIB_FIFO_POLLING_H_

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

//
// Error messages for read and write functions
//
#define ERROR_BUS_BUSY              0x1000
#define ERROR_STOP_NOT_READY        0x5555
#define ERROR_NACK_RECEIVED         0x2000
#define ERROR_TIMEOUT               0x3000
#define ERROR_BUFFER_MAX            0x4000
#define SUCCESS                     0x0000

//
// I2C Library Defines
//
#define MAX_BUFFER_SIZE             64
#define I2C_FIFO_LEVEL              4

//
// Typedefs
//
struct I2CHandle
{
    Uint16 SlaveAddr;           // Slave address tied to the message.
    Uint16 *pControlBuffer;     // Pointer to control bytes buffer
    Uint16 NumOfControlBytes;   // Number of control bytes in message
    Uint16 *pMsgBuffer;         // Pointer to message buffer
    Uint16 NumOfDataBytes;      // Number of valid data bytes in message.
    Uint16 Timeout;             // I2C Timeout variable
};

//
// I2C function prototypes
//
Uint16 I2C_TxSlaveAddress_ControlBytes(struct I2CHandle *I2C_Params);

Uint16 I2C_MasterWrite(struct I2CHandle *I2C_Params);
Uint16 I2C_MasterRead(struct I2CHandle *I2C_Params);
Uint16 WriteByte(Uint8 SlaveAddress, Uint16 RegAddress , Uint16 *data, struct I2CHandle *I2C_Params );
Uint16 ReadBytes(Uint8 SlaveAddress, Uint16 RegAddress , Uint8 count,Uint16 *dest, struct I2CHandle *I2C_Params );


Uint16 checkBusStatus(void);
Uint16 handleNACK(void);
Uint16 handleNACKandTimeout(void);

#endif

//
// End of file
//
