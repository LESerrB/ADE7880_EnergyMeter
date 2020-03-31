/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ade_i2c.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "ade_i2c.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/
#define TRIES   20

ADE_I2C_DATA ade_i2cData;

SYS_TMR_HANDLE delayH;

BYTE __attribute__ ((coherent)) I2C_RxBuffer[7];
BYTE __attribute__ ((coherent)) I2C_TxBuffer[5];

int FailsCount = 0;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

static DRV_I2C_BUFFER_EVENT I2C_WriteReg_x8(WORD reg_address, BYTE reg_dat){
    DRV_I2C_BUFFER_EVENT eventWR = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
        
    switch(ade_i2cData.I2CWR_x8_state){
        case I2CW8_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            /*----- Address -----*/
            I2C_TxBuffer[0] = RegAddress.byte.HB;
            I2C_TxBuffer[1] = RegAddress.byte.LB;
            /*----- Data -----*/
            I2C_TxBuffer[2] = reg_dat;
            
            if(DRV_I2C_Status(sysObj.drvI2C0) == SYS_STATUS_READY){
                ade_i2cData.I2C_BuffHandle = DRV_I2C_Transmit(
                                                        ade_i2cData.I2C_Handle,
                                                        ADE7880_SLAVE_ADDRESS,
                                                        &I2C_TxBuffer[0],
                                                        3,
                                                        NULL
                                                             );
                ade_i2cData.I2CWR_x8_state = I2CW8_STATE_WAIT_REPLY;
            }
        }break;
        
        case I2CW8_STATE_WAIT_REPLY:{
            eventWR = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                               );
            
            if(eventWR == DRV_I2C_BUFFER_EVENT_COMPLETE){
                FailsCount = 0;
                delayH = SYS_TMR_DelayMS(1);
                
                while(!SYS_TMR_DelayStatusGet(delayH))
                    
                ade_i2cData.I2CWR_x8_state = I2CW8_STATE_SEND_CMD;
            }
            else if(eventWR == DRV_I2C_BUFFER_EVENT_PENDING){
                FailsCount++;
                
                if(FailsCount > TRIES){
                    FailsCount = 0;
                    ade_i2cData.I2CWR_x8_state = I2CW8_STATE_SEND_CMD;
                    eventWR = DRV_I2C_SEND_RESTART_EVENT;
                }
            }
        }break;
    }
    
    return eventWR;
}

static DRV_I2C_BUFFER_EVENT I2C_WriteReg_x16(WORD reg_address, WORD reg_dat){
    DRV_I2C_BUFFER_EVENT eventWR = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    WORD_VAL RegDat;
        
    switch(ade_i2cData.I2CWR_x16_state){
        case I2CW16_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            RegDat.Val = reg_dat;
            /*----- Address -----*/
            I2C_TxBuffer[0] = RegAddress.byte.HB;
            I2C_TxBuffer[1] = RegAddress.byte.LB;
            /*----- Data -----*/
            I2C_TxBuffer[2] = RegDat.byte.HB;
            I2C_TxBuffer[3] = RegDat.byte.LB;
            
            if(DRV_I2C_Status(sysObj.drvI2C0) == SYS_STATUS_READY){
                ade_i2cData.I2C_BuffHandle = DRV_I2C_Transmit(
                                                        ade_i2cData.I2C_Handle,
                                                        ADE7880_SLAVE_ADDRESS,
                                                        &I2C_TxBuffer[0],
                                                        4,
                                                        NULL
                                                             );
                ade_i2cData.I2CWR_x16_state = I2CW16_STATE_WAIT_REPLY;
            }
        }break;
        
        case I2CW16_STATE_WAIT_REPLY:{
            eventWR = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                               );
            
            if(eventWR == DRV_I2C_BUFFER_EVENT_COMPLETE){
                FailsCount = 0;
                delayH = SYS_TMR_DelayMS(1);
                
                while(!SYS_TMR_DelayStatusGet(delayH))
                    
                ade_i2cData.I2CWR_x16_state = I2CW16_STATE_SEND_CMD;
            }
            else if(eventWR == DRV_I2C_BUFFER_EVENT_PENDING){
                FailsCount++;
                
                if(FailsCount > TRIES){
                    FailsCount = 0;
                    ade_i2cData.I2CWR_x16_state = I2CW16_STATE_SEND_CMD;
                    eventWR = DRV_I2C_SEND_RESTART_EVENT;
                }
            }
        }break;
    }
    
    return eventWR;
}

static DRV_I2C_BUFFER_EVENT I2C_WriteReg_x32(WORD reg_address, DWORD reg_dat){
    DRV_I2C_BUFFER_EVENT eventWR = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    DWORD_VAL RegDat;
        
    switch(ade_i2cData.I2CWR_x32_state){
        case I2CW32_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            RegDat.Val = reg_dat;
            /*----- Address -----*/
            I2C_TxBuffer[0] = RegAddress.byte.HB;
            I2C_TxBuffer[1] = RegAddress.byte.LB;
            /*----- Data -----*/
            I2C_TxBuffer[2] = RegDat.byte.MB;
            I2C_TxBuffer[3] = RegDat.byte.UB;
            I2C_TxBuffer[4] = RegDat.byte.HB;
            I2C_TxBuffer[5] = RegDat.byte.LB;
            
            if(DRV_I2C_Status(sysObj.drvI2C0) == SYS_STATUS_READY){
                ade_i2cData.I2C_BuffHandle = DRV_I2C_Transmit(
                                                        ade_i2cData.I2C_Handle,
                                                        ADE7880_SLAVE_ADDRESS,
                                                        &I2C_TxBuffer[0],
                                                        6,
                                                        NULL
                                                             );
                ade_i2cData.I2CWR_x32_state = I2CW32_STATE_WAIT_REPLY;
            }
        }break;
        
        case I2CW32_STATE_WAIT_REPLY:{
            eventWR = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                               );
            
            if(eventWR == DRV_I2C_BUFFER_EVENT_COMPLETE){
                FailsCount = 0;
                delayH = SYS_TMR_DelayMS(1);
                
                while(!SYS_TMR_DelayStatusGet(delayH))
                    
                ade_i2cData.I2CWR_x32_state = I2CW32_STATE_SEND_CMD;
            }
            else if(eventWR == DRV_I2C_BUFFER_EVENT_PENDING){
                FailsCount++;
                
                if(FailsCount > TRIES){
                    FailsCount = 0;
                    ade_i2cData.I2CWR_x32_state = I2CW32_STATE_SEND_CMD;
                    eventWR = DRV_I2C_SEND_RESTART_EVENT;
                }
            }
        }break;
    }
    
    return eventWR;
}

static DRV_I2C_BUFFER_EVENT I2C_ReadReg_x8(WORD reg_address){    
    DRV_I2C_BUFFER_EVENT RRevent = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    
    switch(ade_i2cData.I2CRR_x8_state){        
        case I2CR8_STATE_SEND_CMD:{
            I2C_TxBuffer[0] = reg_address;
//            RegAddress.Val = reg_address;
            /*----- Address -----*/
//            I2C_TxBuffer[0] = RegAddress.byte.HB;
//            I2C_TxBuffer[1] = RegAddress.byte.LB;
            
            if (DRV_I2C_Status(sysObj.drvI2C0) == SYS_STATUS_READY){
                ade_i2cData.I2C_BuffHandle = DRV_I2C_TransmitThenReceive(
                                                        ade_i2cData.I2C_Handle, // Handle for ADE7880
                                                        RTCC_SLAVE_ADDRESS,                                
                                                        //ADE7880_SLAVE_ADDRESS,  // Address of the device
                                                        &I2C_TxBuffer[0],       // Tx Buffer
                                                        1,                      // Register size
                                                        &I2C_RxBuffer[0],       // Rx Buffer
                                                        1,                      // Read size register
                                                        NULL                    // Not implemented yet. Always NULL
                                                                        );
                ade_i2cData.I2CRR_x8_state = I2CR8_STATE_WAIT_REPLY;
            }
        }break;
            
        case I2CR8_STATE_WAIT_REPLY:{
            RRevent = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                                );
            
            if(RRevent == DRV_I2C_BUFFER_EVENT_COMPLETE){
                FailsCount = 0;
                delayH = SYS_TMR_DelayMS(1);
                
                while(!SYS_TMR_DelayStatusGet(delayH))
                
                ade_i2cData.I2CRR_x8_state = I2CR8_STATE_SEND_CMD;
            }
            else if(RRevent == DRV_I2C_BUFFER_EVENT_PENDING){
                FailsCount++;
                
                if(FailsCount > TRIES){
                    FailsCount = 0;
                    ade_i2cData.I2CRR_x8_state = I2CR8_STATE_SEND_CMD;
                    RRevent = DRV_I2C_SEND_RESTART_EVENT;
                }
            }
        }break;
    }
    
    return RRevent;
}

static DRV_I2C_BUFFER_EVENT I2C_ReadReg_x16(WORD reg_address){    
    DRV_I2C_BUFFER_EVENT RRevent = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    
    switch(ade_i2cData.I2CRR_x16_state){        
        case I2CR16_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            /*----- Address -----*/
            I2C_TxBuffer[0] = RegAddress.byte.HB;
            I2C_TxBuffer[1] = RegAddress.byte.LB;
            
            if (DRV_I2C_Status(sysObj.drvI2C0) == SYS_STATUS_READY){
                ade_i2cData.I2C_BuffHandle = DRV_I2C_TransmitThenReceive(
                                                        ade_i2cData.I2C_Handle, // Handle for ADE7880
                                                        ADE7880_SLAVE_ADDRESS,  // Address of the device
                                                        &I2C_TxBuffer[0],       // Tx Buffer
                                                        2,                      // Register size
                                                        &I2C_RxBuffer[0],       // Rx Buffer
                                                        2,                      // Read size register
                                                        NULL                    // Not implemented yet. Always NULL
                                                                        );
                ade_i2cData.I2CRR_x16_state = I2CR16_STATE_WAIT_REPLY;
            }
        }break;
            
        case I2CR16_STATE_WAIT_REPLY:{
            RRevent = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                                );
            
            if(RRevent == DRV_I2C_BUFFER_EVENT_COMPLETE){
                FailsCount = 0;
                delayH = SYS_TMR_DelayMS(1);
                
                while(!SYS_TMR_DelayStatusGet(delayH))
                
                ade_i2cData.I2CRR_x16_state = I2CR16_STATE_SEND_CMD;
            }
            else if(RRevent == DRV_I2C_BUFFER_EVENT_PENDING){
                FailsCount++;
                
                if(FailsCount > TRIES){
                    FailsCount = 0;
                    ade_i2cData.I2CRR_x16_state = I2CR16_STATE_SEND_CMD;
                    RRevent = DRV_I2C_SEND_RESTART_EVENT;
                }
            }
        }break;
    }
    
    return RRevent;
}

static DRV_I2C_BUFFER_EVENT I2C_ReadReg_x32(WORD reg_address){    
    DRV_I2C_BUFFER_EVENT RRevent = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    
    switch(ade_i2cData.I2CRR_x32_state){        
        case I2CR32_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            /*----- Address -----*/
            I2C_TxBuffer[0] = RegAddress.byte.HB;
            I2C_TxBuffer[1] = RegAddress.byte.LB;
            
            if (DRV_I2C_Status(sysObj.drvI2C0) == SYS_STATUS_READY){
                ade_i2cData.I2C_BuffHandle = DRV_I2C_TransmitThenReceive(
                                                        ade_i2cData.I2C_Handle, // Handle for ADE7880
                                                        ADE7880_SLAVE_ADDRESS,  // Address of the device
                                                        &I2C_TxBuffer[0],       // Tx Buffer
                                                        2,                      // Register size
                                                        &I2C_RxBuffer[0],       // Rx Buffer
                                                        4,                      // Read size register
                                                        NULL                    // Not implemented yet. Always NULL
                                                                        );
                ade_i2cData.I2CRR_x32_state = I2CR32_STATE_WAIT_REPLY;
            }
        }break;
            
        case I2CR32_STATE_WAIT_REPLY:{
            RRevent = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                                );
            
            if(RRevent == DRV_I2C_BUFFER_EVENT_COMPLETE){
                FailsCount = 0;
                delayH = SYS_TMR_DelayMS(1);
                
                while(!SYS_TMR_DelayStatusGet(delayH))
                
                ade_i2cData.I2CRR_x32_state = I2CR32_STATE_SEND_CMD;
            }
            else if(RRevent == DRV_I2C_BUFFER_EVENT_PENDING){
                FailsCount++;
                
                if(FailsCount > TRIES){
                    FailsCount = 0;
                    ade_i2cData.I2CRR_x32_state = I2CR32_STATE_SEND_CMD;
                    RRevent = DRV_I2C_SEND_RESTART_EVENT;
                }
            }
        }break;
    }
    
    return RRevent;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ADE_I2C_Initialize ( void )

  Remarks:
    See prototype in ade_i2c.h.
 */

void ADE_I2C_Initialize(void){
    /* Place the App state machine in its initial state. */
    ade_i2cData.state = ADE_I2C_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
//    ade_i2cData.HWR_state = ENTRY_RST_MODE;
    ade_i2cData.I2CWR_x8_state = I2CW8_STATE_SEND_CMD;
    ade_i2cData.I2CRR_x8_state = I2CR8_STATE_SEND_CMD;
    ade_i2cData.I2CWR_x16_state = I2CW16_STATE_SEND_CMD;
    ade_i2cData.I2CRR_x16_state = I2CR16_STATE_SEND_CMD;
    ade_i2cData.I2CWR_x32_state = I2CW32_STATE_SEND_CMD;
    ade_i2cData.I2CRR_x32_state = I2CR32_STATE_SEND_CMD;
    ade_i2cData.hsdc_enabled = FALSE;
}


/******************************************************************************
  Function:
    void ADE_I2C_Tasks ( void )

  Remarks:
    See prototype in ade_i2c.h.
 */

void ADE_I2C_Tasks(void){
    DRV_I2C_BUFFER_EVENT event;

    /* Check the application's current state. */
    switch(ade_i2cData.state){
        /* Application's initial state. */
        case ADE_I2C_STATE_INIT:{
            ade_i2cData.I2C_Handle = DRV_I2C_Open(
                                                DRV_I2C_INDEX_0,
                                                DRV_IO_INTENT_READWRITE |
                                                DRV_IO_INTENT_NONBLOCKING |
                                                DRV_IO_INTENT_SHARED
                                                 );
                                
            if(ade_i2cData.I2C_Handle != DRV_HANDLE_INVALID){
                LED_2On();
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_TASKS;
            }
        }break;

        case ADE_I2C_STATE_SERVICE_TASKS:{
            /* Lock I2C as active serial port */
            event = I2C_WriteReg_x8(CONFIG2, 0x02);
            
            if(event == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_TEST0;
            }
            else if(event == DRV_I2C_SEND_RESTART_EVENT){
                DRV_I2C_Close(ade_i2cData.I2C_Handle);
                ade_i2cData.state = ADE_I2C_STATE_INIT;
            }
        }break;

        /* TODO: implement your application state machine.*/
        case ADE_I2C_STATE_SERVICE_TEST0:{
            LED_1On();
            LED_2Off();
        }break;
        
        /* The default state should never be executed. */
        default:
            /* TODO: Handle error in application's state machine. */
        break;
    }
}

 

/*******************************************************************************
 End of File
 */
