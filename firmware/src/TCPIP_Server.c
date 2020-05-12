/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    tcpip_server.c

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

#include "tcpip_server.h"
#include "system_definitions.h"
#if !defined(SYS_CMD_ENABLE)
#include "system/command/sys_command.h"
#endif
#include "system/console/sys_console.h"

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

TCPIP_SERVER_DATA tcpip_serverData;

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
    void TCPIP_SERVER_Initialize ( void )

  Remarks:
    See prototype in tcpip_server.h.
 */

void TCPIP_SERVER_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    tcpip_serverData.state = MOUNT_DISK;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void TCPIP_SERVER_Tasks ( void )

  Remarks:
    See prototype in tcpip_server.h.
 */

void TCPIP_SERVER_Tasks(void){
    SYS_STATUS          tcpipStat;
    TCPIP_NET_HANDLE    netH;
    int                 nNets;
    static IPV4_ADDR    dwLastIP[2] = { {-1}, {-1} };
    IPV4_ADDR           ipAddr;
    int                 i;
    const char          *netName, *netBiosName;
    static uint32_t     startTick=0;
    /* Check the application's current state. */
    switch(tcpip_serverData.state){
        case MOUNT_DISK:
            if(SYS_FS_Mount(SYS_FS_SD_VOL, LOCAL_WEBSITE_PATH_FS, FAT, 0, NULL) == 0){
                SYS_CONSOLE_PRINT("SYS_Initialize: The %s File System is mounted.\r\n", SYS_FS_FATFS_STRING);
                tcpip_serverData.state = TCPIP_WAIT_INIT;
            }
            else{
                //SYS_CONSOLE_PRINT("SYS_Initialize: Mount the %s File System: pending! \r\n", SYS_FS_FATFS_STRING);
            }
        break;

        case TCPIP_WAIT_INIT:
            tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
            
            if(tcpipStat < 0){
            // some error occurred
                SYS_CONSOLE_MESSAGE(" APP: TCP/IP stack initialization failed!\r\n");
                tcpip_serverData.state = TCPIP_ERROR;
            }
            
            else if(tcpipStat == SYS_STATUS_READY){
            // now that the stack is ready we can check the
            // available interfaces and register
            // a Bonjour service

                nNets = TCPIP_STACK_NumberOfNetworksGet();

                for(i = 0; i < nNets; i++){
                    netH = TCPIP_STACK_IndexToNet(i);
                    netName = TCPIP_STACK_NetNameGet(netH);
                    netBiosName = TCPIP_STACK_NetBIOSName(netH);

//#if defined(TCPIP_STACK_USE_NBNS)
//                    SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS enabled\r\n", netName, netBiosName);
//#else
//                    SYS_CONSOLE_PRINT("    Interface %s on host %s - NBNS disabled\r\n", netName, netBiosName);
//#endif  // defined(TCPIP_STACK_USE_NBNS)
//
//#if defined (TCPIP_STACK_USE_ZEROCONF_MDNS_SD)
//                    char mDNSServiceName[] = "MyWebServiceNameX "; // base name of the service Must not exceed 16 bytes long
//                    // the last digit will be incremented by interface
//
//                    mDNSServiceName[sizeof(mDNSServiceName) - 2] = '1' + i;
//                    TCPIP_MDNS_ServiceRegister( netH
//                            , mDNSServiceName                   // name of the service
//                            ,"_http._tcp.local"                 // type of the service
//                            ,80                                 // TCP or UDP port, at which this service is available
//                            ,((const uint8_t *)"path=/index.htm")  // TXT info
//                            ,1                                  // auto rename the service when if needed
//                            ,NULL                               // no callback function
//                            ,NULL);                             // no application context
//#endif //TCPIP_STACK_USE_ZEROCONF_MDNS_SD
                }

                tcpip_serverData.state = TCPIP_TRANSACT;
            }

            break;

        case TCPIP_TRANSACT:
            // if the IP address of an interface has changed
            // display the new value on the system console
            nNets = TCPIP_STACK_NumberOfNetworksGet();
            if(SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet()/5ul){
                startTick = SYS_TMR_TickCountGet();
                LED_1Toggle();
            }

            
            for (i = 0; i < nNets; i++){
                netH = TCPIP_STACK_IndexToNet(i);
                
                if(!TCPIP_STACK_NetIsReady(netH)){
                    return;    // interface not ready yet!
                }
                
                ipAddr.Val = TCPIP_STACK_NetAddress(netH);
                
                if(dwLastIP[i].Val != ipAddr.Val){
                    dwLastIP[i].Val = ipAddr.Val;
                    SYS_CONSOLE_MESSAGE(TCPIP_STACK_NetNameGet(netH));
                    SYS_CONSOLE_MESSAGE(" IP Address: ");
                    SYS_CONSOLE_PRINT("%d.%d.%d.%d \r\n", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
                }
            }
            SYS_CMD_READY_TO_READ();
        break;
        
        /* The default state should never be executed. */
        default:
            /* TODO: Handle error in application's state machine. */
        break;
    }
}

 

/*******************************************************************************
 End of File
 */
