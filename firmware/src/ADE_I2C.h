/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    ade_i2c.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _ADE_I2C_H
#define _ADE_I2C_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "GenericTypeDefs.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
    
/* I2C Address */
#define RTCC_SLAVE_ADDRESS              0xDE
#define ADE7880_SLAVE_ADDRESS           0x70
/* ADE7880 Registers */
#define CONFIG2                         0xEC01
#define HSDC_CFG                        0xE706
#define CONFIG                          0xE618
#define RUN                             0xE228
#define LOCK_RAM1                       0xE7FE
#define LOCK_RAM2                       0xE7E3
    
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum{
	/* Application's state machine's initial state. */
	ADE_I2C_STATE_INIT = 0,
	ADE_I2C_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */
    ADE_I2C_STATE_SERVICE_IDLE,
    ADE_I2C_STATE_SERVICE_TEST0,
    ADE_I2C_STATE_SERVICE_TEST1,
    ADE_I2C_STATE_SERVICE_TEST2,
    ADE_I2C_STATE_SERVICE_TEST3,
    ADE_I2C_STATE_SERVICE_TEST4,
    ADE_I2C_STATE_SERVICE_TEST5,
    ADE7880_STATE_SERVICE_TEST6,
    ADE7880_STATE_SERVICE_TEST7,
    ADE7880_STATE_SERVICE_TEST8,
    ADE7880_STATE_SERVICE_TEST9,
    ADE7880_STATE_SERVICE_TEST10,
    ADE7880_STATE_SERVICE_TEST11,
}ADE_I2C_STATES;

// *****************************************************************************
/* Functions states
 * Summary:
 *  Functions states enumeration
 *
 * Description:
 *  Functions for the Hardware Reset, I2C Read Register and the I2C Write
 *  Register
 */

typedef enum{
    ENTRY_RST_MODE = 0,
    RST_PROCESS,
    RST_COMPLETE,
}HWReset_State;

typedef enum{
    I2CW8_STATE_SEND_CMD = 0,
    I2CW8_STATE_WAIT_REPLY,
}I2C_WriteReg_x8_State;

typedef enum{
    I2CW16_STATE_SEND_CMD = 0,
    I2CW16_STATE_WAIT_REPLY,
}I2C_WriteReg_x16_State;

typedef enum{
    I2CW32_STATE_SEND_CMD = 0,
    I2CW32_STATE_WAIT_REPLY,
}I2C_WriteReg_x32_State;

typedef enum{
    I2CR8_STATE_SEND_CMD = 0,
    I2CR8_STATE_WAIT_REPLY,
}I2C_ReadReg_x8_State;

typedef enum{
    I2CR16_STATE_SEND_CMD = 0,
    I2CR16_STATE_WAIT_REPLY,
}I2C_ReadReg_x16_State;

typedef enum{
    I2CR32_STATE_SEND_CMD = 0,
    I2CR32_STATE_WAIT_REPLY,
}I2C_ReadReg_x32_State;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct{
    /* The application's current state */
    ADE_I2C_STATES state;

    /* TODO: Define any additional data used by the application. */
    HWReset_State               HWR_state;                                      // State for the Hardware Reset application
    I2C_WriteReg_x8_State       I2CWR_x8_state;                                 // State for the I2C 8-Bit register Write function
    I2C_ReadReg_x8_State        I2CRR_x8_state;                                 // State for the I2C 8-Bit register Read function
    I2C_WriteReg_x16_State      I2CWR_x16_state;                                // State for the I2C 16-Bit register Write function
    I2C_ReadReg_x16_State       I2CRR_x16_state;                                // State for the I2C 16-Bit register Read function
    I2C_WriteReg_x32_State      I2CWR_x32_state;                                // State for the I2C 32-Bit register Write function
    I2C_ReadReg_x32_State       I2CRR_x32_state;                                // State for the I2C 32-Bit register Read function

    /* TODO: Define any additional data used by the application. */
    DRV_HANDLE                  I2C_Handle;                                     // Handle for ADE7880 I2C communication
    
    DRV_I2C_BUFFER_HANDLE       I2C_BuffHandle;                                 // Handle for I2C Buffer communication
    
    BOOL                        hsdc_enabled;                                   // Indicates the status of the HSDC communication
} ADE_I2C_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ADE_I2C_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    ADE_I2C_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void ADE_I2C_Initialize(void);


/*******************************************************************************
  Function:
    void ADE_I2C_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    ADE_I2C_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void ADE_I2C_Tasks(void);


#endif /* _ADE_I2C_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

