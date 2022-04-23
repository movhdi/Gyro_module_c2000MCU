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
void   I2CA_Init(void);

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

    //Sample code to just communicate with the sensor
    //first initialize the i2c handle structure
    ControlBuffer[0] = 0x0000;
    ControlBuffer[0] = WHO_AM_I_MPU9250;
    Gyro.NumOfControlBytes = 1;
    Gyro.NumOfDataBytes = 1;
    //Gyro.SlaveAddr = SlaveAddress_I2C;
    Gyro.pControlBuffer = ControlBuffer;
    Gyro.pMsgBuffer = RxMsgBuffer;
    Gyro.Timeout = 100;

    Gyro.SlaveAddr = 0x68;
    while(1)
        {
/*        for(i=0x0; i<0x7F ; i++)
        {
            // Delay between read attempts during EEPROM write cycle time
            Gyro.SlaveAddr = i;
            DELAY_US(1000);

            Status = I2C_MasterRead(&Gyro);

            if(Status == SUCCESS)
            {
                k++;
                True_address = (&Gyro)->SlaveAddr;
            }
        }*/
        //k=0;


        DELAY_US(1000);
            /*//I2caRegs.I2CFFTX.all = 0x6000;
            I2caRegs.I2CCNT = 1;
            //
            // Set up as master transmitter
            // FREE + MST + TRX + IRS
            //
            I2caRegs.I2CMDR.all = 0x4620;
            I2caRegs.I2CSAR = 0x68;
            I2caRegs.I2CDXR = WHO_AM_I_MPU9250; //Register address transmit
            I2caRegs.I2CMDR.bit.STT = 0x1; // Send START condition
            //Checking ACK or NACK
                while(I2caRegs.I2CSTR.bit.ARDY == 1) // Waiting for FIFO to be empty again
                                                   // Here it takes 1 Byte of data (8 clks +1 clk ACK)
                                                   // Every clock takes 2.5 us, 9clks takes 22.5 us
                {
                    DELAY_US(10);
                }
                if(I2caRegs.I2CSTR.bit.NACK == 1)
                    {
                        // Clear NACK
                        I2caRegs.I2CSTR.bit.NACK = 1;

                        // disable FIFO
                        //I2caRegs.I2CFFTX.all = 0x0000;
                        //I2caRegs.I2CFFRX.all = 0x0000;

                        // Generate STOP condition
                        I2caRegs.I2CMDR.bit.STP = 1;
                    }
                //I2caRegs.I2CFFTX.bit.TXFFRST = 0;

                //Transmitting the read session
                I2caRegs.I2CCNT = 1;
                // Enable RX FIFO
                //I2caRegs.I2CFFRX.all = 0x2040;

                //
                // Set up as master receiver
                // FREE + MST + IRS
                //
                I2caRegs.I2CMDR.all = 0x4420;

                I2caRegs.I2CMDR.bit.STT = 0x1; // Send repeated START condition
                while(I2caRegs.I2CSTR.bit.ARDY == 1)
                {
                    DELAY_US(10);
                }
                RxMsgBuffer[0] = I2caRegs.I2CDRR;
                I2caRegs.I2CMDR.bit.STP = 1;*/

        Status = I2C_MasterRead(&Gyro);

        if(Status == SUCCESS)
        {
            k++;
            True_address = (&Gyro)->SlaveAddr;
        }
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
