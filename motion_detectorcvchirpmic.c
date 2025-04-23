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



// Constants for timing
#define CAM_HOLD_MS         500    // keep camera detection visible for 0.5 seconds
#define MIC_DEBOUNCE_SAMPLES 8     // 8 consecutive samples > threshold
#define MIC_HOLD_MS         300    // keep "Mic" banner 0.3 seconds
#define MIC_CHIRP_PERIOD_MS 250    // at most 4 chirps per second

#define SOUND_THRESHOLD 517  // Adjused this based on testing with microphone

int32_t GUIwake;                   // semaphore for waking GUI immediately

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
    static uint32_t camEndTime = 0;    // timestamp when detection should end
    static uint32_t nextChirpTime = 0; // timestamp for rate-limiting chirps
    static uint16_t yieldCounter = 0;  // counter to occasionally yield control
    
    while(1) {
        int c = UART1_ReceiveChar();
        
        if(c != -1) {  // If data received
            // Echo character back for terminal feedback
            UART1_SendChar((uint8_t)c);
            
            // Check if it's a camera detection signal (1 or 0)
            if (c == '1') {
                // Set detection flag and end time
                Saw = 1;
                camEndTime = Time + CAM_HOLD_MS;
                OS_Signal(&GUIwake);  // Wake GUI immediately
                
                // Play detection chirp if enough time has passed
                if (Time >= nextChirpTime) {
                    BSP_Buzzer_Set(512);  
                    BSP_Delay1ms(80);
                    BSP_Buzzer_Set(0);
                    nextChirpTime = Time + MIC_CHIRP_PERIOD_MS; // Rate-limit chirps
                }
            }
            else if (c == '0') {
                Saw = 0;  // Clear camera detection flag
                camEndTime = 0;
                OS_Signal(&GUIwake);  // Wake GUI immediately
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
        
        // Check if camera detection timer has expired
        if (Saw && Time >= camEndTime) {
            Saw = 0;  // Clear flag after hold time expires
            OS_Signal(&GUIwake);  // Wake GUI immediately
        }
        
        // Occasionally let other threads run by yielding
        yieldCounter++;
        if (yieldCounter >= 100) {  // Every ~100 passes
            yieldCounter = 0;
            BSP_Delay1ms(1);  // Short delay to allow other threads to run
        }
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
// *********Task1*********
// collects data from microphone and performs sound threshold detection
// Inputs:  none
// Outputs: none
void Task1(void){
    static int32_t soundSum = 0;
    static int time = 0;  // units of microphone sampling rate
    static uint16_t aboveCnt = 0;  // count of samples above threshold
    static uint32_t micEndTime = 0;   // timestamp when mic detection should end
    static uint32_t nextChirpTime = 0;  // timestamp for next chirp
    static uint16_t yieldCounter = 0;  // counter to occasionally yield control
    
    while(1){
        Profile_Toggle0(); 
        BSP_Microphone_Input(&SoundData);
        soundSum = soundSum + (int32_t)SoundData;
        SoundArray[time] = SoundData;
        time++;
        
        // Sound threshold detection when in Microphone mode
        if(PlotState == Microphone) {
            // Check if sound is above threshold
            if(SoundData > SOUND_THRESHOLD) {
                aboveCnt++;
                // First time we consider it "detected" after debounce period
                if(aboveCnt == MIC_DEBOUNCE_SAMPLES) {
                    Heard = 1;
                    But1 = 0;  // Simulate button press for UI
                    micEndTime = Time + MIC_HOLD_MS;
                    OS_Signal(&GUIwake);  // Wake GUI immediately
                    
                    // Play chirp if enough time has passed
                    if(Time >= nextChirpTime) {
                        BSP_Buzzer_Set(512);
                        BSP_Delay1ms(80);
                        BSP_Buzzer_Set(0);
                        nextChirpTime = Time + MIC_CHIRP_PERIOD_MS;
                    }
                }
            } else {
                aboveCnt = 0;  // Reset counter if below threshold
            }
            
            // Check if mic detection timer has expired
            if (Heard && Time >= micEndTime) {
                Heard = 0;
                But1 = 1;  // Reset simulated button press
                OS_Signal(&GUIwake);  // Wake GUI when clearing
            }
        } else {
            // Reset detection when not in mic mode
            if(Heard) {
                Heard = 0;
                But1 = 1;
                aboveCnt = 0;
                OS_Signal(&GUIwake);  // Wake GUI when clearing
            }
        }
        
        // Process full buffer of samples
        if(time == SOUNDRMSLENGTH){
            SoundAvg = soundSum/SOUNDRMSLENGTH;
            OS_FIFO_Put(SoundAvg);
            soundSum = 0;
            OS_Signal(&NewData);
            time = 0;
        }
        
        // Occasionally let other threads run by yielding
        yieldCounter++;
        if (yieldCounter >= 100) {  // Every ~100 passes
            yieldCounter = 0;
            BSP_Delay1ms(1);  // Short delay to allow other threads to run
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
        // Wait for GUI wake signal
        OS_Wait(&GUIwake);
        
        if(ReDrawAxes){
            drawaxes();
            ReDrawAxes = 0;
        }
        
        OS_Wait(&LCDmutex);
        
        // No detection case
        if(But1 != 0 && Saw == 0 && Heard == 0){
            BSP_LCD_PlotPoint(30, NOCOLOR);
            BSP_LCD_FillRect(25, 40, 90, 40, LCD_BLACK);
        } 
        // Camera detection case - prioritize if both detected
        else if(Saw == 1 || (But1 == 0 && PlotState == Cam)){
            BSP_LCD_PlotPoint(150, PERSONCOLOR);
            BSP_LCD_DrawString(5, 5, "Person",  PERSONCOLOR);
            BSP_LCD_DrawString(5, 6, "Detected",  PERSONCOLOR);
            BSP_LCD_DrawString(5, 7, "Camera",  PERSONCOLOR);
        }
        // Microphone detection case
        else if(Heard == 1 || (But1 == 0 && PlotState == Microphone)){
            BSP_LCD_PlotPoint(SoundData/10, SOUNDCOLOR);
            BSP_LCD_PlotPoint(150, HEARDCOLOR);
            BSP_LCD_DrawString(5, 5, "Person",  HEARDCOLOR);
            BSP_LCD_DrawString(5, 6, "Detected",  HEARDCOLOR);
            BSP_LCD_DrawString(5, 7, "Mic",  HEARDCOLOR);
        }
        
        BSP_LCD_PlotIncrement();
        OS_Signal(&LCDmutex);
        
        // Also process the mailbox to prevent overflow
        data = OS_MailBox_Recv();
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
    Profile_Init();
    Task0_Init();
    Task1_Init();
    BSP_LCD_Init();
    BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
    BSP_LCD_Drawaxes(PERSONCOLOR, BGCOLOR, "Time", "", 0,"", 0, 500, 0);
    BSP_LCD_DrawString(0,1, "Security Sys",  PERSONCOLOR);
    
    Time = 0;
    OS_InitSemaphore(&NewData, 0);
    OS_InitSemaphore(&LCDmutex, 1);
    OS_InitSemaphore(&GUIwake, 0);  // Initialize GUI wake semaphore
    OS_MailBox_Init();
    OS_FIFO_Init();
    
    OS_AddThreads(&Task2, &Task3, &Task0, &Task1);
    
    // Initialize hardware
    LED_Init();
    LED_Set(LED_RED);
    Delay(1000000);
    UART1_Init();
    LED_Set(LED_GREEN);
    UART1_SendString("\r\nTIVA C Command Processor Ready\r\n");
    
    // Launch OS
    OS_Launch(BSP_Clock_GetFreq()/THREADFREQ);
    
    return 0;  // Never reaches here
}
 