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
// Application Name     - SPI Demo
// Application Overview - The demo application focuses on showing the required
//                        initialization sequence to enable the CC3200 SPI
//                        module in full duplex 4-wire master and slave mode(s).
// Application Details  -
// http://processors.wiki.ti.com/index.php/CC32xx_SPI_Demo
// or
// docs\examples\CC32xx_SPI_Demo.pdf
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <string.h>
#include <stdio.h>
// Driverlib includes
#include "hw_types.h"
#include "hw_uart.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
//#include "rom_map.c"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "gpio.h"
#include "udma.h"
#include "udma_if.h"
#include "timer_if.h"
#include "timer_if.c"

#include "uart_if.h"
#include "gpio_if.h"
#include "i2c_if.h"
// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"

//adafruit files
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "test.h"
#include "math.h"
#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1//set as master

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

//#define ZERO 0x165E9A
#define ZERO 0x6022822A88288;
#define ONE 0x17EE81
#define TWO 0x161E9E
#define THREE 0x151EAE
#define FOUR 0x171E8E
#define FIVE 0x149EB6
#define SIX 0x169E96
#define SEVEN 0x159EA6
#define EIGHT 0x179E86
#define NINE 0x165EBA
#define MUTE 0x173E8C

#define IR_PIN 0x40
#define IR_BASE GPIOA0_BASE
//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

volatile char messageOutBuffer[52];
volatile char messageInBuffer[52];
volatile int messageInBufferIndex=0;
volatile int messageOutBufferIndex;
volatile char currentOutChar;
volatile int letterOffset;
volatile int timeOfLastButtonPress;
volatile int timerCounter;
volatile int last;
volatile int current;
volatile int interruptCounter;
volatile int timeWhenLastButtonPressed = 0;
volatile static tBoolean bRxDone;
int x_cursorTx = 0;
int y_cursorTx = 0;
int x_cursorRx = 0;
int y_cursorRx = 70;
char tmp;
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
static void IRIntHandler(void){//rising and falling edge
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus(IR_BASE,true);
    MAP_GPIOIntClear(IR_BASE,ulStatus);
    last = current;
    current = timerCounter;
    interruptCounter++;

}

int convertToButton(char values[52]){
    int buttonPressed=-1;
    if(values[12] == '0' && values[14] == '1' && values[16] == '1' && values[18] =='0'){//second hex value 6
        if(values[20] == '0' && values[22] == '1' && values[24] == '0' && values[26] == '1'){//third hex value 5
                buttonPressed = 0;
        }
        else if(values[20] == '0' && values[22] == '0' && values[24] == '0' && values[26]=='1')//third hex value 1
            buttonPressed = 2;
        else{
            if(values[44] == '0' && values[46] == '1' && values[48] == '1' && values[50] =='0')
                buttonPressed = 6;
        }


    }
    else if(values[4] == '1')
        buttonPressed = 11;
    else if(values[12]=='0' && values[14]=='1' && values[16]=='0' && values[18] =='0'){//second hex value 4
        if(values[20]=='1' && values[22]=='0' &&values[24]=='0' && values[26]=='1')//third hex value 9
            buttonPressed = 5;
        else
            if(values[44]=='1' && values[46]=='0' && values[48] == '1' && values[50] == '0')
                buttonPressed = 9;
    }
    else if(values[12] == '0' && values[14] =='1' && values[16] =='1' && values[18]=='1'){//second hex value 7
        if(values[20]=='1' && values[22]=='1' && values[24]=='1' && values[26]=='0')//third hex value E
           buttonPressed = 1;
        else if(values[20]=='0' && values[22]=='0' && values[24]=='0' && values[26]=='1')//third hex value 1
            buttonPressed = 4;
        else if(values[20]=='1' && values[22]=='0' && values[24]=='0' && values[26]=='1')//third hex value 9
            buttonPressed = 8;
        else
            buttonPressed = 10;//mute
    }
    else if(values[12] == '0' && values[14] == '1' && values[16] == '0' && values[18] =='1'){//second hex value 5
        if(values[20]=='0' && values[22]=='0' && values[24]=='0' && values[26]=='1')//third hex value 1
            buttonPressed = 3;
        else
            buttonPressed = 7;
    }
    else
        buttonPressed = -1;

    return buttonPressed;

}

void timerPulseWidthIntHandler(void){
    Timer_IF_InterruptClear(TIMERA0_BASE);
    timerCounter++;

}
void printCharTop(char character){


    if(character == 30){//delete a character
        x_cursorTx -= 6;
        fillRect(x_cursorTx,y_cursorTx, 6, 8, BLACK);
        x_cursorTx -=6;
    }
    else if(character == 31){//button pressed =1, wipe screen
        messageOutBufferIndex = 0;
        fillScreen(BLACK);
        x_cursorTx = 0;
        y_cursorTx = 0;
    }
    else
        drawChar(x_cursorTx,y_cursorTx,character,WHITE,BLACK,1);
}

void printCharBottom(char character){
    drawChar(x_cursorRx,y_cursorRx,character,WHITE,BLACK,1);

    x_cursorRx += 6;
    if(x_cursorRx >= 124){
        x_cursorRx = 0;
        y_cursorRx += 8;
    }

}
volatile unsigned char UARTDataBuffer[128];
volatile unsigned char UARTData;
volatile unsigned int UARTDataCount = 0;
unsigned long intStatus;
volatile int UARTDataFlag =0;
char UART_STATUS;
static void UARTRxIntHandler()
{

      UART_STATUS = UARTIntStatus(UARTA1_BASE, true);

      if((UART_STATUS & UART_INT_RX) && MAP_UARTCharsAvail(UARTA1_BASE))
      {
        //clear the receive interrupt
        MAP_UARTIntClear(UARTA1_BASE, UART_INT_RX);
        //receive data here
        UARTDataBuffer[UARTDataCount] = (unsigned char)MAP_UARTCharGetNonBlocking(UARTA1_BASE);
        UARTDataCount++;
        UARTDataFlag = 1;
        //printCharBottom(UARTData);
       }
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    unsigned long ulStatus;
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    //MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);
    //
    // Reset SPI/
    //
    MAP_SPIReset(GSPI_BASE);
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    SPICSEnable(GPIOA0_BASE);
    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    //Oled initialize
    Adafruit_Init();
    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();
    bRxDone = false;
    //fillScreen(YELLOW);
    //Setup Uart
    /*
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                                UART_BAUD_RATE,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                UART_CONFIG_PAR_NONE));
                                */
    //UDMAInit();
    MAP_UARTConfigSetExpClk(UARTA1_BASE,MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                      /*UART_BAUD_RATE*/9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
    //fillScreen(GREEN);
    //MAP_UARTIntRegister(UARTA1_BASE,UARTRxIntHandler);
    //fillScreen(BLACK);
    //MAP_UARTIntEnable(UARTA1_BASE,UART_INT_RX);
    //fillScreen(RED);
    //MAP_UARTDMAEnable(UARTA1_BASE,UART_DMA_TX);
    //fillScreen(BLUE);
    //MAP_UARTFlowControlSet(UARTA1_BASE, UART_FLOWCONTROL_NONE);
    MAP_UARTFIFODisable(UARTA1_BASE);
    MAP_UARTIntRegister(UARTA1_BASE, UARTRxIntHandler);
    //MAP_UARTIntEnable(UARTA0_BASE, UART_INT_RX);
    MAP_UARTIntEnable(UARTA1_BASE, UART_INT_RX|UART_INT_RT);

    //Setup Timer for waveform
    Timer_IF_Init(PRCM_TIMERA0,TIMERA0_BASE,TIMER_CFG_PERIODIC,TIMER_A,0);
    Timer_IF_IntSetup(TIMERA0_BASE,TIMER_A,timerPulseWidthIntHandler);
    TimerLoadSet(TIMERA0_BASE,TIMER_A,MILLISECONDS_TO_TICKS(.5));



    //Setup interrupt for GPIO pin of IR Reciever
    MAP_GPIOIntRegister(IR_BASE,IRIntHandler);
    MAP_GPIOIntTypeSet(IR_BASE,IR_PIN,0x00000001);//GPIO_BOTH_EDGES
    ulStatus = MAP_GPIOIntStatus(IR_BASE,false);
    MAP_GPIOIntClear(IR_BASE,ulStatus);
    MAP_GPIOIntEnable(IR_BASE,IR_PIN);
    MAP_TimerEnable(TIMERA0_BASE,TIMER_A);
    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t********************************************\n\r");
    Message("\t\t        LAB 3 IR Receiver  \n\r");
    Message("\t\t********************************************\n\r");
    Message("\n\n\n\r");
    timerCounter = 0;
    current = 0;
    last = 0;
    interruptCounter = 0;
    messageOutBufferIndex = 0;
    letterOffset = 0;
    timeOfLastButtonPress = 0;
    messageOutBufferIndex = 0;
    int lastInterruptCounter = 0;
    int length;
    char buffer[52];
    //int buffer = 0;
    int bufferCount=0;
    int buttonPressed;
    fillScreen(BLACK);
    while(1){
        if(UARTDataFlag==1){
            int i;
            fillRect(0,64,128,64,BLACK);
            for(i =1;i<UARTDataCount;i++)
                printCharBottom(UARTDataBuffer[i]);
            UARTDataCount = 0;
            UARTDataFlag = 0;
            x_cursorRx = 0;
            UARTDataCount = 0;//may need to remove
        }
        if(interruptCounter != lastInterruptCounter){//If an interrupt happened, figure out if 0 or 1
            lastInterruptCounter = interruptCounter;
            length = current -last;
            bufferCount++;

            if(length > 10){
                //reset bufferCount/save time by not clearing buffer, just overwrite it and reset bufferCount
                bufferCount = 0;
            }
            else if(length >= 4){
                buffer[bufferCount] = '1';

            }
            else
                buffer[bufferCount] = '0';
            /*
            else if(length < 4){
                buffer += 0;
            }
            */
            if(bufferCount == 51 && (timerCounter-timeWhenLastButtonPressed)>500){//captured a full waveform
                //convert to a button
                timeWhenLastButtonPressed = timerCounter;
                if(timerCounter-timeOfLastButtonPress > 1600){//Confirm a Letter
                            letterOffset = 0;
                            //x_cursorTx += 6;
                            x_cursorTx += 6;
                                if(x_cursorTx >= 124){
                                    x_cursorTx =0;
                                    y_cursorTx += 8;
                                }
                            if(currentOutChar != 30){
                                messageOutBuffer[messageOutBufferIndex] = currentOutChar;
                                messageOutBufferIndex++;
                            }
                        }
                timeOfLastButtonPress = timerCounter;
                buttonPressed = convertToButton(buffer);

                if(buttonPressed == 2){//ABC
                    if(letterOffset ==3)
                          letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,97+letterOffset,WHITE,BLACK,1);
                    printCharTop(97+letterOffset);
                    currentOutChar = 97+letterOffset;
                    Report("hh\r\n\n");
                }
                else if(buttonPressed == 3){//DEF
                    if(letterOffset ==3)
                          letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,100+letterOffset,WHITE,BLACK,1);
                    printCharTop(100+letterOffset);
                    currentOutChar = 100+letterOffset;
                }
                else if(buttonPressed == 4){//GHI
                    if(letterOffset ==3)
                          letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,103+letterOffset,WHITE,BLACK,1);
                    printCharTop(103+letterOffset);
                    currentOutChar = 103+letterOffset;
                }
                else if(buttonPressed == 5){//JKL
                    if(letterOffset ==3)
                          letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,106+letterOffset,WHITE,BLACK,1);
                    printCharTop(106+letterOffset);
                    currentOutChar = 106+letterOffset;
                }
                else if(buttonPressed == 6){//MNO
                    if(letterOffset ==3)
                         letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,109+letterOffset,WHITE,BLACK,1);
                    printCharTop(109+letterOffset);
                    currentOutChar = 109+letterOffset;
                }
                else if(buttonPressed == 7){//PQRS
                    if(letterOffset ==4)
                         letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,112+letterOffset,WHITE,BLACK,1);
                    printCharTop(112+letterOffset);
                    currentOutChar = 112+letterOffset;
                }
                else if(buttonPressed == 8){//TUV
                    if(letterOffset ==3)
                         letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,116+letterOffset,WHITE,BLACK,1);
                    printCharTop(116+letterOffset);
                    currentOutChar = 116+letterOffset;
                }
                else if(buttonPressed == 9){//WXYZ
                    if(letterOffset ==4)
                        letterOffset = 0;
                    //drawChar(x_cursorTx,y_cursorTx,119+letterOffset,WHITE,BLACK,1);
                    printCharTop(119+letterOffset);
                    currentOutChar = 119+letterOffset;
                }
                else if(buttonPressed == 0){//SPACE
                    //drawChar(x_cursorTx,y_cursorTx,94,WHITE,BLACK,1);
                    printCharTop(32);
                    currentOutChar = 32;
                }
                else if(buttonPressed == 10){//MUTE(delete)
                    printCharTop(30);
                    currentOutChar = 30;
                    //subtract 1 from current buffer location
                    messageOutBufferIndex-=1;
                }
                else if(buttonPressed ==11){//Send
                    int i;

                    for (i = 0; i < messageOutBufferIndex; i++) {
                        //UARTCharPut(UARTA1_BASE, messageOutBuffer[i]);
                        MAP_UARTCharPut(UARTA1_BASE,messageOutBuffer[i]);
                        //Report("%c",messageOutBuffer[i]);
                    }
                    //Report("\n\r");
                    //clear the top half of the oled display
                    fillRect(0, 0, 127, 63, BLACK);
                    x_cursorTx = 0;
                    y_cursorTx = 0;
                    messageOutBufferIndex = 0;

                }
                else if(buttonPressed ==1){//clear screen
                    printCharTop(31);
                }
                letterOffset++;
                //Report("Button Pressed: %d\n\r",buttonPressed);
                //int i;
                //for(i = 4; i<=51;i+=2)
                  //  Report("%c",buffer[i]);
                //Report("\r\n\n");
                bufferCount = 0;

                //buffer = 0;
            }

            //Report("Length %d\n",current-last);
        }

        /*
        if(timerCounter >400){
            timerCounter = 0;
        }
        */
    }


    //
    // Reset the peripheral
    //
    //MAP_PRCMPeripheralReset(PRCM_GSPI);



}
