#ifndef MAIN_H
#define MAIN_H

//funtion prototypes
void I2CA_Init(void);
void InitEPwm3Example(void);
void pass(void);
void fail(void);
// Macros
#define Gyro_READ_ATTEMPTS        10
#define Gyro_DATA_BYTES           32

#define EPWM3_TIMER_TBPRD  2000  // Period register
#define EPWM3_MAX_CMPA      950
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB     1050
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

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



Uint16 k=0;
EPWM_INFO epwm3_info;

#endif
