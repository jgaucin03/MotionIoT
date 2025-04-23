// Lab2.c
// Runs on either MSP432 or TM4C123
// Starter project to Lab 2.  Take sensor readings, process the data,
// and output the results.  Specifically, this program will
// measure steps using the accelerometer, audio sound amplitude using
// microphone, and temperature. (we will add light back in Lab 3)
// Daniel and Jonathan Valvano
// July 12, 2016

/* This example accompanies the books
"Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2016

"Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
ISBN: 978-1466468863, Jonathan Valvano, copyright (c) 2016

"Embedded Systems: Introduction to the MSP432 Microcontroller",
ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2016

"Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2016

Copyright 2016 by Jonathan W. Valvano, valvano@mail.utexas.edu
You may use, edit, run or distribute this file
as long as the above copyright notice remains
THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
For more information about my classes, my research, and my books, see
http://users.ece.utexas.edu/~valvano/
*/


#include <stdint.h>
#include "../inc/BSP.h"
#include "../inc/Profile.h"

#include "../inc/CortexM.h"
#include "os.h"

#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"


#define THREADFREQ 1000   // frequency in Hz of round robin scheduler

//---------------- Global variables shared between tasks ----------------
uint32_t Time;              // elasped time in 100 ms units
uint32_t Steps;             // number of steps counted
uint32_t Magnitude;         // will not overflow (3*1,023^2 = 3,139,587)
                            // Exponentially Weighted Moving Average
uint32_t EWMA;              // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
uint16_t SoundData;         // raw data sampled from the microphone
int32_t SoundAvg;

uint32_t LightData;
int32_t TemperatureData;    // 0.1C

uint8_t But1=1;
uint8_t But2=1;
uint8_t Saw=0;
uint8_t Heard=0;
// semaphores
int32_t NewData;  // true when new numbers to display on top of LCD
int32_t LCDmutex; // exclusive access to LCD
int ReDrawAxes = 0;         // non-zero means redraw axes on next display task

enum plotstate{
    Cam,
    Microphone,
    Temperature
};
enum plotstate PlotState = Cam;
//color constants
#define BGCOLOR     LCD_BLACK
#define AXISCOLOR   LCD_ORANGE
#define MAGCOLOR    LCD_YELLOW
#define PERSONCOLOR    LCD_YELLOW
#define EWMACOLOR   LCD_CYAN
#define SOUNDCOLOR  LCD_CYAN
#define HEARDCOLOR 	LCD_RED
#define TEMPCOLOR   LCD_LIGHTGREEN
#define NOCOLOR LCD_WHITE
#define TOPNUMCOLOR LCD_ORANGE
//------------ end of Global variables shared between tasks -------------




#define SOUND_THRESHOLD 517  // Adjused this based on testing with microphone

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// LED Colors
#define LED_RED     0x02
#define LED_BLUE    0x04
#define LED_GREEN   0x08
#define LED_YELLOW  (LED_RED | LED_GREEN)   // 0x0A
#define LED_PURPLE  (LED_RED | LED_BLUE)    // 0x06
#define LED_CYAN    (LED_GREEN | LED_BLUE)  // 0x0C
#define LED_WHITE   (LED_RED | LED_GREEN | LED_BLUE) // 0x0E
#define LED_OFF     0x00

// Buffer for receiving commands
#define BUFFER_SIZE 32
char rxBuffer[BUFFER_SIZE];
uint8_t rxIndex = 0;

// Initialize onboard LED
void LED_Init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R5) == 0){};
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= 0x0E;
    GPIO_PORTF_DIR_R |= 0x0E;
    GPIO_PORTF_DEN_R |= 0x0E;
}

// Set LED color
void LED_Set(uint8_t color) {
    GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~0x0E) | color;
}

// Simple delay function
void Delay(uint32_t time) {
    volatile uint32_t i;
    for(i=0; i<time; i++);
}

// Initialize UART1 for communication with HC-05
// PB0 (U1Rx) and PB1 (U1Tx)
void UART1_Init(void) {
    // Enable UART1 and PORTB clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    
    // Wait for the clocks to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R1) == 0){};
    
    // Configure GPIO pins for UART
    GPIO_PORTB_AFSEL_R |= 0x03;                          // Enable alternate function
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) | 0x00000011; // Configure for UART
    GPIO_PORTB_DEN_R |= 0x03;                            // Enable digital I/O
    
    // Configure UART1
    UART1_CTL_R &= ~UART_CTL_UARTEN;                     // Disable UART for setup
    
    // Configure for 9600 baud rate
    // Assumes 16MHz system clock (no PLL)
    UART1_IBRD_R = 520;                                  // Integer part (16MHz/16/9600)
    UART1_FBRD_R = 53;                                   // Fractional part
    
    // 8-bit data, 1 stop bit, no parity
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;     // 8-bit, FIFOs enabled
    UART1_CTL_R |= UART_CTL_UARTEN;                      // Enable UART
}
 
// Send a character via UART1
void UART1_SendChar(uint8_t data) {
    while((UART1_FR_R & UART_FR_TXFF) != 0);             // Wait until FIFO not full
    UART1_DR_R = data;                                   // Send data
}
 
 // Send a string via UART1
void UART1_SendString(char *str) {
    while(*str) {
        UART1_SendChar((uint8_t)*str);
        str++;
    }
}
 
// Receive a character via UART1 (non-blocking)
int UART1_ReceiveChar(void) {
    if((UART1_FR_R & UART_FR_RXFE) != 0)                 // Check if RX FIFO empty
        return -1;                                        // Return -1 if no data
    return UART1_DR_R & 0xFF;                            // Return received data
}
 
// Process command in the buffer
void processCommand(void) {
     // Null terminate the string
     rxBuffer[rxIndex] = '\0';
     
     // Check command type
     if (strcmp(rxBuffer, "PING") == 0) {
         // Echo back "PONG\r\n"
         UART1_SendString("PONG\r\n");
         LED_Set(LED_WHITE);
         Delay(500000);
         LED_Set(LED_GREEN);
     }
     else if (strcmp(rxBuffer, "STATUS") == 0) {
         // Send status information
         UART1_SendString("TIVA C Status: OK\r\n");
         UART1_SendString("Bluetooth: Connected\r\n");
         UART1_SendString("System: Ready\r\n");
     }
     else if (strcmp(rxBuffer, "HELP") == 0) {
         // Send help information
         UART1_SendString("Available Commands:\r\n");
         UART1_SendString("PING - Test connection\r\n");
         UART1_SendString("LED:R/G/B/Y/P/C/W/OFF - Control LED\r\n");
         UART1_SendString("ECHO:text - Echo text back\r\n");
         UART1_SendString("STATUS - System status\r\n");
         UART1_SendString("HELP - Show this help\r\n");
     }
     else if (strncmp(rxBuffer, "LED:", 4) == 0) {
         // LED control command
         char color = rxBuffer[4];
         switch(color) {
             case 'R':
                 LED_Set(LED_RED);
                 UART1_SendString("LED set to Red\r\n");
                 break;
             case 'G':
                 LED_Set(LED_GREEN);
                 UART1_SendString("LED set to Green\r\n");
                 break;
             case 'B':
                 LED_Set(LED_BLUE);
                 UART1_SendString("LED set to Blue\r\n");
                 break;
             case 'Y':
                 LED_Set(LED_YELLOW);
                 UART1_SendString("LED set to Yellow\r\n");
                 break;
             case 'P':
                 LED_Set(LED_PURPLE);
                 UART1_SendString("LED set to Purple\r\n");
                 break;
             case 'C':
                 LED_Set(LED_CYAN);
                 UART1_SendString("LED set to Cyan\r\n");
                 break;
             case 'W':
                 LED_Set(LED_WHITE);
                 UART1_SendString("LED set to White\r\n");
                 break;
             default:
                 if (strncmp(rxBuffer+4, "OFF", 3) == 0) {
                     LED_Set(LED_OFF);
                     UART1_SendString("LED turned off\r\n");
                 } else {
                     UART1_SendString("Unknown LED color\r\n");
                 }
                 break;
         }
     }
     else if (strncmp(rxBuffer, "ECHO:", 5) == 0) {
         // Echo command - echo back the text after "ECHO:"
         UART1_SendString(rxBuffer + 5);
         UART1_SendString("\r\n");
     }
     else {
         // Unknown command
         UART1_SendString("Unknown command: ");
         UART1_SendString(rxBuffer);
         UART1_SendString("\r\n");
     }
     
     // Reset buffer index
     rxIndex = 0;
}







//---------------- Task0 samples sound from microphone ----------------
// Event thread run by OS in real time at 1000 Hz

// *********Task0_Init*********
// initializes microphone
// Task0 measures sound intensity
// Inputs:  none
// Outputs: none
void Task0_Init(void){
 
}
 
 
 
// *********Task0*********
// Periodic event thread runs in real time at 1000 Hz
// handles UART communication and manages camera detection signals
// Inputs:  none
// Outputs: none
void Task0(void){
    uint16_t DetCount = 0;
    uint16_t ChirpCount = 0;
    
    while(1) {
        int c = UART1_ReceiveChar();
        
        if(c != -1) {  // If data received
            // Echo character back for terminal feedback
            UART1_SendChar((uint8_t)c);
            
            // Check if it's a camera detection signal (1 or 0)
            if (c == '1') {
                DetCount = 0;
                if (Saw == 0 || ChirpCount >= 5) {
                    // Play detection chirp
                    BSP_Buzzer_Set(512);  
                    ChirpCount = 0;
                    BSP_Delay1ms(100);
                }
                Saw = 1;  // Set camera detection flag
            }
            else if (c == '0') {
                Saw = 0;  // Clear camera detection flag
            }
            // Process other commands
            else if (c == '\r' || c == '\n') {
                if (rxIndex > 0) {
                    UART1_SendString("\r\n");
                    processCommand();
                }
            }
            else if (rxIndex < BUFFER_SIZE - 1) {
                rxBuffer[rxIndex++] = (char)c;
            }
        }
        
        // Manage detection timers
        if (DetCount > 3)
            Saw = 0;  // Auto-clear camera detection after timeout
        
        DetCount++;
        ChirpCount++;
        BSP_Buzzer_Set(0);  // Turn off buzzer after chirp
    }
}

/* ****************************************** */
/*          End of Task0 Section              */
/* ****************************************** */
#define SOUNDRMSLENGTH 1000 // number of samples to collect before calculating RMS (may overflow if greater than 4104)
int16_t SoundArray[SOUNDRMSLENGTH];
// *********Task1_Init*********
// initializes accelerometer
// Task1 counts Steps
// Inputs:  none
// Outputs: none
void Task1_Init(void){
    BSP_Microphone_Init();
}
// *********Task1*********
// collects data from microphone and performs sound threshold detection
// Inputs:  none
// Outputs: none
void Task1(void){
    uint32_t squared;
    static int32_t soundSum = 0;
    static int time = 0; // units of microphone sampling rate
    static bool soundDetected = false;
    static int soundCounter = 0;     // For debouncing sound detection
    static int soundHoldCounter = 0; // For maintaining detection state
    static int chirpCounter = 0;     // For sound detection chirp
    
    while(1){
        Profile_Toggle0(); 
        BSP_Microphone_Input(&SoundData);
        soundSum = soundSum + (int32_t)SoundData;
        SoundArray[time] = SoundData;
        time = time + 1;
        
        // Sound threshold detection when in Microphone mode
        if(PlotState == Microphone) {
            // Check if sound is above threshold
            if(SoundData > SOUND_THRESHOLD) {
                soundCounter++;
                // Require multiple consecutive samples above threshold
                if(soundCounter > 8) {
                    // If we weren't already detecting sound, play chirp
                    if(!soundDetected && chirpCounter >= 5) {
                        BSP_Buzzer_Set(512);
                        BSP_Delay1ms(50);
                        BSP_Buzzer_Set(0);
                        chirpCounter = 0;
                    }
                    soundDetected = true;
                    Heard = 1;       // Set global flag
                    But1 = 0;        // Simulate button press for UI
                    soundHoldCounter = 0; // Reset hold timeout
                }
            } else {
                soundCounter = 0;
                if(soundDetected) {
                    // Keep detection on for a while even after sound drops
                    soundHoldCounter++;
                    if(soundHoldCounter > 500) { // ~0.5 second hold
                        soundDetected = false;
                        Heard = 0;
                        But1 = 1;    // Release simulated button
                    }
                }
            }
            
            chirpCounter++; // Increment chirp counter
        }
        
        // Reset to camera input when switching modes
        if(PlotState != Microphone) {
            if(soundDetected) {
                soundDetected = false;
                Heard = 0;
                But1 = 1;  // Release simulated button
            }
        }
        
        if(time == SOUNDRMSLENGTH){
            SoundAvg = soundSum/SOUNDRMSLENGTH;
            OS_FIFO_Put(SoundAvg);
            soundSum = 0;
            OS_Signal(&NewData);
            time = 0;
        }
    }
}

/* ****************************************** */
/*          End of Task1 Section              */
/* ****************************************** */


//---------------- Task2 calculates steps and plots data on LCD ----------------
// Main thread scheduled by OS round robin preemptive scheduler
// accepts data from accelerometer, calculates steps, plots on LCD, and output to LED
// If no data are lost, the main loop in Task2 runs exactly at 10 Hz, but not in real time
enum state{                 // the step counting algorithm cycles through four states
    LookingForMax,            // looking for a local maximum in current magnitude
    LookingForCross1,         // looking for current magnitude to cross average magnitude, minus a constant
    LookingForMin,            // looking for a local minimum in current magnitude
    LookingForCross2          // looking for current magnitude to cross average magnitude, plus a constant
};
enum state AlgorithmState = LookingForMax;
#define LOCALCOUNTTARGET 5  // The number of valid measured magnitudes needed to confirm a local min or local max.  Increase this number for longer strides or more frequent measurements.
#define AVGOVERSHOOT 25     // The amount above or below average a measurement must be to count as "crossing" the average.  Increase this number to reject increasingly hard shaking as steps.
#define ACCELERATION_MAX 1400
#define ACCELERATION_MIN 600
#define ALPHA 128           // The degree of weighting decrease, a constant smoothing factor between 0 and 1,023. A higher ALPHA discounts older observations faster.
                            // basic step counting algorithm is based on a forum post from
                            // http://stackoverflow.com/questions/16392142/android-accelerometer-profiling/16539643#16539643
#define SOUND_MAX 900
#define SOUND_MIN 300
#define LIGHT_MAX 200000
#define LIGHT_MIN 0
#define TEMP_MAX 1023
#define TEMP_MIN 0
void drawaxes(void){
   OS_Wait(&LCDmutex);
   if(0){
     BSP_LCD_Drawaxes(AXISCOLOR, BGCOLOR, "Time", "Mag", MAGCOLOR, "Ave", EWMACOLOR, ACCELERATION_MAX, ACCELERATION_MIN);
   } 
   OS_Signal(&LCDmutex);  ReDrawAxes = 0;
}
void Task2(void){
    uint32_t data;
    drawaxes();
    
    while(1){
        data = OS_MailBox_Recv();

        if(ReDrawAxes){
            drawaxes();
            ReDrawAxes = 0;
        }
        
        OS_Wait(&LCDmutex);
        
        // No detection
        if(But1 != 0 && Saw == 0 && Heard == 0){
            BSP_LCD_PlotPoint(30, NOCOLOR);
            BSP_LCD_FillRect(25, 40, 90, 40, LCD_BLACK);
        } 
        // Camera detection
        else if((But1 == 0 && PlotState == Cam) || Saw == 1){
            BSP_LCD_PlotPoint(150, PERSONCOLOR);
            BSP_LCD_DrawString(5, 5, "Person",  PERSONCOLOR);
            BSP_LCD_DrawString(5, 6, "Detected",  PERSONCOLOR);
            BSP_LCD_DrawString(5, 7, "Camera",  PERSONCOLOR);
            BSP_Delay1ms(25);
        }
        // Microphone detection
        else if((But1 == 0 && PlotState == Microphone) || Heard == 1){
            BSP_LCD_PlotPoint(SoundData/10, SOUNDCOLOR);
            BSP_LCD_PlotPoint(150, HEARDCOLOR);
            BSP_LCD_DrawString(5, 5, "Person",  HEARDCOLOR);
            BSP_LCD_DrawString(5, 6, "Detected",  HEARDCOLOR);
            BSP_LCD_DrawString(5, 7, "Mic",  HEARDCOLOR);
            BSP_Delay1ms(25);
        }
        
        BSP_LCD_PlotIncrement();
        OS_Signal(&LCDmutex);
    }
}

/* ****************************************** */
/*          End of Task2 Section              */
/* ****************************************** */


//------------Task3 handles switch input, buzzer output, LED output-------
// *********Task3*********
// Main thread scheduled by OS round robin preemptive scheduler
// non-real-time task
// checks the switches, updates the mode, and outputs to the buzzer 
// Inputs:  none
// Outputs: none
void Task3(void){
    static uint8_t prev1 = 0, prev2 = 0;

    BSP_Button1_Init();
    BSP_Button2_Init();
    BSP_Buzzer_Init(0);
    BSP_RGB_Init(0, 0, 0);

    // Initial mode display
    OS_Wait(&LCDmutex);
    BSP_LCD_DrawString(0, 0, "Mode: CAM", PERSONCOLOR);
    OS_Signal(&LCDmutex);

    while(1){
        Profile_Toggle3();
        BSP_Buzzer_Set(0);
        But1 = BSP_Button2_Input();
        
        if((But1 == 0) && (prev1 != 0)){
            BSP_Buzzer_Set(512);
        }
        prev1 = But1;
        
        But2 = BSP_Button1_Input();
        if((But2 == 0) && (prev2 != 0)){
            // Toggle between Camera and Microphone modes
            if(PlotState == Cam){
                PlotState = Microphone;
                OS_Wait(&LCDmutex);
                BSP_LCD_FillRect(0, 0, 90, 20, LCD_BLACK);
                BSP_LCD_DrawString(0, 0, "Mode: MIC", SOUNDCOLOR);
                OS_Signal(&LCDmutex);
                
                // Reset detection flags when switching modes
                Saw = 0;
            } else if(PlotState == Microphone){
                PlotState = Cam;
                OS_Wait(&LCDmutex);
                BSP_LCD_FillRect(0, 0, 90, 20, LCD_BLACK);
                BSP_LCD_DrawString(0, 0, "Mode: CAM", PERSONCOLOR);
                OS_Signal(&LCDmutex);
                
                // Reset detection flags when switching modes
                Heard = 0;
            }
            BSP_Buzzer_Set(512);
        }
        prev2 = But2;
        
        BSP_Delay1ms(5);
    }
}
    
/* ****************************************** */
/*          End of Task3 Section              */
/* ****************************************** */




/* ****************************************** */
/*          End of Task5 Section              */
/* ****************************************** */


//***************Step 4*************************
// Increase to 4 threads
int main(void){
    OS_Init();
    Profile_Init();  // initialize the 7 hardware profiling pins
    Task0_Init();    // microphone init
    Task1_Init();    // accelerometer init
    BSP_LCD_Init();
        BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
        BSP_LCD_Drawaxes(PERSONCOLOR, BGCOLOR, "Time", "", 0,"", 0, 500, 0);
        BSP_LCD_DrawString(0,1, "Security Sys",  PERSONCOLOR);
    Time = 0;
    OS_InitSemaphore(&NewData, 0);  // 0 means no data
    OS_InitSemaphore(&LCDmutex, 1); // 1 means free
    OS_MailBox_Init();              // initialize mailbox used to send data between Task1 and Task2
        OS_FIFO_Init();
        // Tasks 0, 1 will not run
    // Task2, Task3, Task4, Task5 are main threads
    // Tasks 2 and 5 will stall
    OS_AddThreads(&Task2, &Task3, &Task0,&Task1);
        
        
            // Initialize systems
        LED_Init();
        
        // Initially set LED to red to show we're starting
        LED_Set(LED_RED);
        Delay(1000000);
        
        // Initialize UART1 at 9600 baud (no PLL, using 16MHz clock)
        UART1_Init();
        
        // Show ready state
        LED_Set(LED_GREEN);
        
        // Send startup message
        UART1_SendString("\r\nTIVA C Command Processor Ready\r\n");
        
        

    //	
        
    // when grading change 1000 to 4-digit number from edX
    OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here
        
        
        
    return 0;             // this never executes
}
 