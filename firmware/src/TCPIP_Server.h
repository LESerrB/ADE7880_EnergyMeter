/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    tcpip_server.h

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

#ifndef _TCPIP_SERVER_H
#define _TCPIP_SERVER_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/int/sys_int.h"
#include "system/ports/sys_ports.h"
#include "system/fs/sys_fs_media_manager.h"
#include "system/fs/fat_fs/src/hardware_access/diskio.h"
#include "system/fs/fat_fs/src/file_system/ff.h"
#include "system/fs/mpfs/mpfs.h"
#include "system/fs/sys_fs.h"
#include "system/debug/sys_debug.h"
#include "tcpip/src/common/sys_fs_wrapper.h"
#include "tcpip/tcpip.h"
#include "driver/ethmac/drv_ethmac.h"
#include "driver/nvm/drv_nvm.h"
#include "driver/tmr/drv_tmr.h"

extern const uint8_t MPFS_IMAGE_DATA[];

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

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    /* The app mounts the disk */
    MOUNT_DISK = 0,

    /* In this state, the application waits for the initialization of the TCP/IP stack
     * to complete. */
    TCPIP_WAIT_INIT,

    /* In this state, the application can do TCP/IP transactions. */
    TCPIP_TRANSACT,

    /* The application waits in this state for the driver to be ready
       before sending the "hello world" message. */
    //APP_STATE_WAIT_FOR_READY,

    /* The application waits in this state for the driver to finish
       sending the message. */
    //APP_STATE_WAIT_FOR_DONE,

    /* The application does nothing in the idle state. */
    STATE_IDLE,

    //
    USERIO_LED_DEASSERTED,

    USERIO_LED_ASSERTED,

    TCPIP_ERROR,
} TCPIP_SERVER_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    TCPIP_SERVER_STATES state;

    /* TODO: Define any additional data used by the application. */
    SYS_FS_HANDLE           fileHandle;
} TCPIP_SERVER_DATA;


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
    void TCPIP_SERVER_Initialize ( void )

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
    TCPIP_SERVER_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void TCPIP_SERVER_Initialize ( void );


/*******************************************************************************
  Function:
    void TCPIP_SERVER_Tasks ( void )

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
    TCPIP_SERVER_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void TCPIP_SERVER_Tasks( void );


#endif /* _TCPIP_SERVER_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
