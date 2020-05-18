/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ade_hsdc.c

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

#include "ade_hsdc.h"

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

ADE_HSDC_DATA ade_hsdcData;

int SampCount;

/*signed long*/ int BigBuffer[20000];

char buffer[32];                                                                // Buffer that stores the mount name of uSD Card

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


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ADE_HSDC_Initialize ( void )

  Remarks:
    See prototype in ade_hsdc.h.
 */

void ADE_HSDC_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ade_hsdcData.state = ADE_HSDC_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    SampCount = 0;
}


/******************************************************************************
  Function:
    void ADE_HSDC_Tasks ( void )

  Remarks:
    See prototype in ade_hsdc.h.
 */

void ADE_HSDC_Tasks(void){
    static uint32_t startTick = 0;
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_PENDING;
    
    /* Check the application's current state. */
    switch(ade_hsdcData.state){
        /* Application's initial state. */
        case ADE_HSDC_STATE_INIT:{
            ade_hsdcData.SPIHandle = DRV_SPI_Open(DRV_SPI_INDEX_1,
                                                  DRV_IO_INTENT_READWRITE);
            
            if(ade_hsdcData.SPIHandle != NULL){
                ade_hsdcData.state = ADE_HSDC_STATE_SERVICE_TASKS;
            }            
        }break;

        case ADE_HSDC_STATE_SERVICE_TASKS:{
            if(ade_i2cData.hsdc_enabled == true){
                LED_2Off();
                ade_hsdcData.state = ADE_HSDC_STATE_CAPT_DATA;
            }
        }break;

        /* TODO: implement your application state machine.*/
        case ADE_HSDC_STATE_CAPT_DATA:{
            ade_hsdcData.Buffer_Handle = DRV_SPI_BufferAddRead(ade_hsdcData.SPIHandle,
                                                               (SPI_DATA_TYPE*)&ade_hsdcData.RXbuffer[0],
                                                               1,
                                                               NULL,
                                                               NULL);
            
            ade_hsdcData.state = ADE_HSDC_STATE_WAIT_REQUEST;
        }break;
        
        case ADE_HSDC_STATE_WAIT_REQUEST:{
            event = DRV_SPI_BufferStatus(ade_hsdcData.Buffer_Handle);
            
            if(event == DRV_SPI_BUFFER_EVENT_COMPLETE){
                BigBuffer[SampCount] = ade_hsdcData.RXbuffer[0];
                SampCount++;
                
                if(SampCount < 20000)
                    ade_hsdcData.state = ADE_HSDC_STATE_CAPT_DATA;
                else
                    ade_hsdcData.state = ADE_HSDC_STATE_CHECK_SD;
            }
        }break;
        
        case ADE_HSDC_STATE_CHECK_SD:{
            if(SYS_FS_CurrentDriveGet(buffer) == SYS_FS_RES_SUCCESS){           // Checks if the volume is attached
               ade_hsdcData.state = ADE_HSDC_STATE_OPEN_DIR;
            }
        }break;
        
        case ADE_HSDC_STATE_OPEN_DIR:{
            ade_hsdcData.dirHandle = SYS_FS_DirOpen("Muestras Rapidas");
            
            if(ade_hsdcData.dirHandle != SYS_FS_HANDLE_INVALID)
                SYS_FS_DirClose(ade_hsdcData.dirHandle);
            else 
                SYS_FS_DirectoryMake("Muestras Rapidas");
            
            ade_hsdcData.fileHandle = SYS_FS_FileOpen("Muestras Rapidas/QS_01.txt", (SYS_FS_FILE_OPEN_WRITE));
            
            if(ade_hsdcData.fileHandle != SYS_FS_HANDLE_INVALID){
                //SampCount = 0;
                ade_hsdcData.state = ADE_HSDC_STATE_SAVE_DATA;
            }
        }break;
        
        case ADE_HSDC_STATE_SAVE_DATA:{
            for(SampCount = 0; SampCount < 400; SampCount++){
                SYS_FS_FilePrintf(ade_hsdcData.fileHandle, "%d", BigBuffer[SampCount]);
                SYS_FS_FilePrintf(ade_hsdcData.fileHandle, "%d", BigBuffer[SampCount + 1]);
                SYS_FS_FilePrintf(ade_hsdcData.fileHandle, "%d", BigBuffer[SampCount + 2]);
                SYS_FS_FilePrintf(ade_hsdcData.fileHandle, "%d", BigBuffer[SampCount + 3]);                
            }
            
            SYS_FS_FileClose(ade_hsdcData.fileHandle);
            ade_hsdcData.state = ADE_HSDC_STATE_SERVICE_IDLE;
        }break;
        
        case ADE_HSDC_STATE_SERVICE_IDLE:{
            if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet()/10ul){
                startTick = SYS_TMR_TickCountGet();
                LED_2Toggle();
            }
        }break;

        /* The default state should never be executed. */
        default:{
            /* TODO: Handle error in application's state machine. */
        }break;
    }
}

 

/*******************************************************************************
 End of File
 */
