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
SYS_TMR_HANDLE delayH;

BYTE __attribute__ ((coherent)) I2C_RxBuffer[5];
BYTE __attribute__ ((coherent)) I2C_TxBuffer[5];
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
static BOOL ADE7880_HWReset(void){
    BOOL ST = FALSE;
    
    switch(ade_i2cData.HWR_state){
        case ENTRY_RST_MODE:{
            ADE_RSTOff();
            ade_i2cData.HWR_state = RST_PROCESS;
            delayH = SYS_TMR_DelayMS(10000);
        }break;
        
        case RST_PROCESS:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                ADE_RSTOn();
                ade_i2cData.HWR_state = RST_COMPLETE;
            }
        }break;
        
        case RST_COMPLETE:{
            if(!ADE_IRQ1StateGet()){
                ST = TRUE;
                ade_i2cData.HWR_state = ENTRY_RST_MODE;
            }
        }break;
    }
    
    return ST;
}

static DRV_I2C_BUFFER_EVENT I2C_WriteReg_x8(uint16_t dev_address, WORD reg_address, BYTE reg_dat){
    DRV_I2C_BUFFER_EVENT eventWR = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    int numdat;

    switch(ade_i2cData.I2CWR_x8_state){
        case I2CW8_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            
            /*----- Register Address -----*/
            if(dev_address == RTCC_SLAVE_ADDRESS){
                I2C_TxBuffer[0] = RegAddress.byte.LB;
            /*----- Register Data -----*/
                I2C_TxBuffer[1] = reg_dat;
            /*--- Transmicion Data ----*/
                numdat = 2;
            }
            else if(dev_address == ADE7880_SLAVE_ADDRESS){
                I2C_TxBuffer[0] = RegAddress.byte.HB;
                I2C_TxBuffer[1] = RegAddress.byte.LB;
            /*----- Register Data -----*/
                I2C_TxBuffer[2] = reg_dat;
            /*--- Transmicion Data ----*/
                numdat = 3;
            }

            ade_i2cData.I2C_BuffHandle = DRV_I2C_Transmit(
                                                    ade_i2cData.I2C_Handle,
                                                    dev_address,
                                                    &I2C_TxBuffer[0],
                                                    numdat,
                                                    NULL
                                                         );
            ade_i2cData.I2CWR_x8_state = I2CW8_STATE_WAIT_REPLY;
        }break;
        
        case I2CW8_STATE_WAIT_REPLY:{
            eventWR = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                               );
            
            if(eventWR == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.I2CWR_x8_state = I2CW8_STATE_SEND_CMD;
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
            
            ade_i2cData.I2C_BuffHandle = DRV_I2C_Transmit(
                                                    ade_i2cData.I2C_Handle,
                                                    ADE7880_SLAVE_ADDRESS,
                                                    &I2C_TxBuffer[0],
                                                    4,
                                                    NULL
                                                         );
            ade_i2cData.I2CWR_x16_state = I2CW16_STATE_WAIT_REPLY;
        }break;
        
        case I2CW16_STATE_WAIT_REPLY:{
            eventWR = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                               );
            
            if(eventWR == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.I2CWR_x16_state = I2CW16_STATE_SEND_CMD;
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
            
            ade_i2cData.I2C_BuffHandle = DRV_I2C_Transmit(
                                                    ade_i2cData.I2C_Handle,
                                                    ADE7880_SLAVE_ADDRESS,
                                                    &I2C_TxBuffer[0],
                                                    6,
                                                    NULL
                                                         );
            ade_i2cData.I2CWR_x32_state = I2CW32_STATE_WAIT_REPLY;
        }break;
        
        case I2CW32_STATE_WAIT_REPLY:{
            eventWR = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                               );
            
            if(eventWR == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.I2CWR_x32_state = I2CW32_STATE_SEND_CMD;
            }
        }break;
    }
    
    return eventWR;
}

static DRV_I2C_BUFFER_EVENT I2C_ReadReg_x8(uint16_t dev_address, WORD reg_address){    
    DRV_I2C_BUFFER_EVENT RRevent = DRV_I2C_BUFFER_EVENT_PENDING;
    WORD_VAL RegAddress;
    int numdat;
    
    switch(ade_i2cData.I2CRR_x8_state){        
        case I2CR8_STATE_SEND_CMD:{
            RegAddress.Val = reg_address;
            /*----- Register Address -----*/
            if(dev_address == RTCC_SLAVE_ADDRESS){
                I2C_TxBuffer[0] = RegAddress.byte.LB;
                numdat = 1;
            }
            else if(dev_address == ADE7880_SLAVE_ADDRESS){
                I2C_TxBuffer[0] = RegAddress.byte.HB;
                I2C_TxBuffer[1] = RegAddress.byte.LB;
                numdat = 2;
            }
            
            ade_i2cData.I2C_BuffHandle = DRV_I2C_TransmitThenReceive(
                                                    ade_i2cData.I2C_Handle,     // Handle for ADE7880                  
                                                    dev_address,                // Address of the device
                                                    &I2C_TxBuffer[0],           // Tx Buffer
                                                    numdat,                     // Register size
                                                    &I2C_RxBuffer[0],           // Rx Buffer
                                                    1,                          // Read size register
                                                    NULL                        // Not implemented yet. Always NULL
                                                                    );
            ade_i2cData.I2CRR_x8_state = I2CR8_STATE_WAIT_REPLY;
        }break;
            
        case I2CR8_STATE_WAIT_REPLY:{
            RRevent = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                                );
            
            if(RRevent == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.I2CRR_x8_state = I2CR8_STATE_SEND_CMD;
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
            
            ade_i2cData.I2C_BuffHandle = DRV_I2C_TransmitThenReceive(
                                                    ade_i2cData.I2C_Handle,     // Handle for ADE7880
                                                    ADE7880_SLAVE_ADDRESS,      // Address of the device
                                                    &I2C_TxBuffer[0],           // Tx Buffer
                                                    2,                          // Register size
                                                    &I2C_RxBuffer[0],           // Rx Buffer
                                                    2,                          // Read size register
                                                    NULL                        // Not implemented yet. Always NULL
                                                                    );
            ade_i2cData.I2CRR_x16_state = I2CR16_STATE_WAIT_REPLY;
        }break;
            
        case I2CR16_STATE_WAIT_REPLY:{
            RRevent = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                                );
            
            if(RRevent == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.I2CRR_x16_state = I2CR16_STATE_SEND_CMD;
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
            
            ade_i2cData.I2C_BuffHandle = DRV_I2C_TransmitThenReceive(
                                                    ade_i2cData.I2C_Handle,     // Handle for ADE7880
                                                    ADE7880_SLAVE_ADDRESS,      // Address of the device
                                                    &I2C_TxBuffer[0],           // Tx Buffer
                                                    2,                          // Register size
                                                    &I2C_RxBuffer[0],           // Rx Buffer
                                                    4,                          // Read size register
                                                    NULL                        // Not implemented yet. Always NULL
                                                                    );
            ade_i2cData.I2CRR_x32_state = I2CR32_STATE_WAIT_REPLY;
        }break;
            
        case I2CR32_STATE_WAIT_REPLY:{
            RRevent = DRV_I2C_TransferStatusGet(
                                              ade_i2cData.I2C_Handle,
                                              ade_i2cData.I2C_BuffHandle
                                                );
            
            if(RRevent == DRV_I2C_BUFFER_EVENT_COMPLETE){
                ade_i2cData.I2CRR_x32_state = I2CR32_STATE_SEND_CMD;
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
    ade_i2cData.HWR_state = ENTRY_RST_MODE;
    ade_i2cData.I2CWR_x8_state = I2CW8_STATE_SEND_CMD;
    ade_i2cData.I2CRR_x8_state = I2CR8_STATE_SEND_CMD;
    ade_i2cData.I2CWR_x16_state = I2CW16_STATE_SEND_CMD;
    ade_i2cData.I2CRR_x16_state = I2CR16_STATE_SEND_CMD;
    ade_i2cData.I2CWR_x32_state = I2CW32_STATE_SEND_CMD;
    ade_i2cData.I2CRR_x32_state = I2CR32_STATE_SEND_CMD;
    ade_i2cData.hsdc_enabled = FALSE;
    
    ade_i2cData.hsdc_enabled = false;
}


/******************************************************************************
  Function:
    void ADE_I2C_Tasks ( void )

  Remarks:
    See prototype in ade_i2c.h.
 */

void ADE_I2C_Tasks(void){
    static uint32_t startTick = 0;
    //DRV_I2C_BUFFER_EVENT event = DRV_I2C_BUFFER_EVENT_PENDING;
    
    switch(ade_i2cData.state){
        case ADE_I2C_STATE_INIT:{
            
            ade_i2cData.I2C_Handle = DRV_I2C_Open(DRV_I2C_INDEX_0,
                                                  DRV_IO_INTENT_READWRITE|
                                                  DRV_IO_INTENT_NONBLOCKING|
                                                  DRV_IO_INTENT_SHARED);
            
            if(ade_i2cData.I2C_Handle != DRV_HANDLE_INVALID){
                LED_1On();
                LED_2On();
                ade_i2cData.state = ADE_I2C_STATE_RST_ADE;
            }
            // O
            // O
        }break;
        
        /* Reset the ADE, needed for properly work after power up */
        case ADE_I2C_STATE_RST_ADE:{
            if(ADE7880_HWReset()){
                LED_1Off();
                LED_2Off();
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_TEST0;
            }
            // -
            // -
        }break;
        
        /* RTC Read (test i2c) */
        case ADE_I2C_STATE_SERVICE_TEST0:{
            if(I2C_ReadReg_x8(RTCC_SLAVE_ADDRESS, 0x03)
                                            == DRV_I2C_BUFFER_EVENT_COMPLETE){
                delayH = SYS_TMR_DelayMS(1);
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_TEST1;
            }
        }break;
        
        case ADE_I2C_STATE_SERVICE_TEST1:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                LED_1On();  
                ade_i2cData.state = ADE_I2C_STATE_LOCK_I2C;
            }
            // -
            // O
        }break;

        /* Lock I2C */
        case ADE_I2C_STATE_LOCK_I2C:{
            if(I2C_WriteReg_x8(ADE7880_SLAVE_ADDRESS, CONFIG2, 0x02)
                                            == DRV_I2C_BUFFER_EVENT_COMPLETE){
                delayH = SYS_TMR_DelayMS(1);
                ade_i2cData.state = ADE_I2C_STATE_DELAY1;
            }
        }break;
        
        case ADE_I2C_STATE_DELAY1:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                LED_2On();
                ade_i2cData.state = ADE_I2C_STATE_CON_HSDC;
            }
            // O
            // O
        }break;
        
        /* Config HSDC */
        case ADE_I2C_STATE_CON_HSDC:{
            if(I2C_WriteReg_x8(ADE7880_SLAVE_ADDRESS, HSDC_CFG, 0x08)
                                            == DRV_I2C_BUFFER_EVENT_COMPLETE){
                delayH = SYS_TMR_DelayMS(1);
                ade_i2cData.state = ADE_I2C_STATE_DELAY2;
            }
        }break;
        
        case ADE_I2C_STATE_DELAY2:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                LED_1Off();
                ade_i2cData.state = ADE_I2C_STATE_EN_HSDC;
            }
            // O
            // -
        }break;
        
        /* Enable HSDC */
        case ADE_I2C_STATE_EN_HSDC:{
            if(I2C_WriteReg_x16(CONFIG, 0x0042)
                                            == DRV_I2C_BUFFER_EVENT_COMPLETE){
                delayH = SYS_TMR_DelayMS(1);
                ade_i2cData.state = ADE_I2C_STATE_DELAY3;
            }
        }break;
        
        case ADE_I2C_STATE_DELAY3:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                LED_2Off();
                ade_i2cData.state = ADE_I2C_STATE_EN_DSP;
            }
            // -
            // -
        }break;
        
        case ADE_I2C_STATE_EN_DSP:{
            if(I2C_WriteReg_x16(RUN, 0x0001)
                                            == DRV_I2C_BUFFER_EVENT_COMPLETE){
                delayH = SYS_TMR_DelayMS(1);
                ade_i2cData.state = ADE_I2C_STATE_DELAY4;
            }
        }break;
        
        case ADE_I2C_STATE_DELAY4:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                LED_1On();
                LED_2On();
                ade_i2cData.hsdc_enabled = true;
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_IDLE;
            }
            // O
            // O
        }break;

        case ADE_I2C_STATE_SERVICE_TEST2:{
            if(I2C_ReadReg_x16(LAST_ADD)
                                            == DRV_I2C_BUFFER_EVENT_COMPLETE){
                delayH = SYS_TMR_DelayMS(1);
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_TEST3;
            }
        }break;
        
        case ADE_I2C_STATE_SERVICE_TEST3:{
            if(SYS_TMR_DelayStatusGet(delayH)){
                LED_2Off();
                ade_i2cData.state = ADE_I2C_STATE_SERVICE_IDLE;
            }
        }break;
        
        case ADE_I2C_STATE_SERVICE_IDLE:{
            if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet()/2ul){
                startTick = SYS_TMR_TickCountGet();
                LED_1Toggle();
//                ade_i2cData.state = ADE_I2C_STATE_SERVICE_TEST2;
            }
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
