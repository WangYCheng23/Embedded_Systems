//*****************************************************************************
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
//
// Application Name     - Blinky
// Application Overview - The objective of this application is to showcase the 
//                        GPIO control using Driverlib api calls. The LEDs 
//                        connected to the GPIOs on the LP are used to indicate 
//                        the GPIO output. The GPIOs are driven high-low 
//                        periodically in order to turn on-off the LEDs.
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_Blinky_Application
// or
// docs\examples\CC32xx_Blinky_Application.pdf
//
//*****************************************************************************

//****************************************************************************
//
//! \addtogroup blinky
//! @{
//
//****************************************************************************

// Standard includes
#include <stdio.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "gpio.h"
#include "utils.h"
#include "uart.h"
// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "pin_mux_config.h"

#define APPLICATION_VERSION     "1.1.1"
//            MACROS
//*****************************************************************************
#define APP_NAME     "GPIO";
#define SW3 4
#define SW2 15
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES                           
//*****************************************************************************
void LEDBlinkyRoutine();
static void BoardInit(void);

//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                         
//*****************************************************************************

//*****************************************************************************
//
//! Configures the pins as GPIOs and peroidically toggles the lines
//!
//! \param None
//! 
//! This function  
//!    1. Configures 3 lines connected to LEDs as GPIO
//!    2. Sets up the GPIO pins as output
//!    3. Periodically toggles each LED one by one by toggling the GPIO line
//!
//! \return None
//
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Message("\n\n\n\r");
    Message("\t\t *************************************************\n\r");
    Message("\t\t        CC3200 GPIO Application       \n\r");
    Message("\t\t *************************************************\n\r");
    Message("\n\n\n\r");
    Message("\n\n\n\r");
    Message("\t\t *************************************************\n\r");
    Message("\t\t        Push SW3 to start LED binary Counting    \n\r");
    Message("\t\t        Push SW2 to blink LEDs on and off    \n\r");
    Message("\t\t *************************************************\n\r");
    Message("\n\n\n\r");
}

void LEDbinaryCounting(int counter){
    switch(counter){
        case 0:
            GPIO_IF_LedOff(MCU_ALL_LED_IND);
            break;
        case 1:
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
            break;
        case 2:
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
            break;
        case 3:
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
            break;
        case 4:
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
            break;
        case 5:
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
            break;
        case 6:
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
            break;
        case 7:
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
            break;
        default:
            GPIO_IF_LedOff(MCU_ALL_LED_IND);
            break;
    }
    MAP_UtilsDelay(8000000);
}
/*void LEDBlinkyRoutine()
{
    //
    // Toggle the lines initially to turn off the LEDs.
    // The values driven are as required by the LEDs on the LP.
    //
    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    while(1)
    {
        //
        // Alternately toggle hi-low each of the GPIOs
        // to switch the corresponding LED on/off.
        //
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        MAP_UtilsDelay(8000000);
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
    }

}*/
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}
//****************************************************************************
//
//! Main function
//!
//! \param none
//! 
//! This function  
//!    1. Invokes the LEDBlinkyTask
//!
//! \return None.
//
//****************************************************************************
int
main()
{
    int counter = 0;
    int state_counting = 0;
    int state_blinking = 0;
    //int blinkingCounter;
    int ledState = 0;
    //
    // Initialize Board configurations
    //
    BoardInit();
    
    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();
    InitTerm();
    ClearTerm();
    DisplayBanner("GPIO");
    GPIO_IF_LedConfigure(LED1|LED2|LED3);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
    
    //
    // Start the LEDBlinkyRoutine
    //
    while(1){
        if(GPIOPinRead(GPIOA1_BASE,0X20)==32){
            if(state_counting == 0)
                Message("SW3 Pressed\n\r");
            GPIOPinWrite(GPIOA3_BASE,0x10,0x00);
            state_counting = 1;
            state_blinking = 0;
            //blinkingCounter = 0;
        }
        if(GPIOPinRead(GPIOA2_BASE,0x40)==64){
            if(state_blinking == 0)
                Message("SW2 Pressed\n\r");
            GPIOPinWrite(GPIOA3_BASE,0x10,0xff);
            state_counting = 0;
            state_blinking = 1;
            counter = 0;
            ledState = 0;
        }

        if(state_counting == 1){
            LEDbinaryCounting(counter);
            counter++;
            if(counter == 8)
                counter = 0;
        }
        if(state_blinking == 1){
            if(ledState == 1){
                ledState = 0;
                GPIO_IF_LedOff(MCU_ALL_LED_IND);
            }
            else{
                ledState = 1;
                GPIO_IF_LedOn(MCU_ALL_LED_IND);
            }
            MAP_UtilsDelay(8000000);
        }
    }

    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
