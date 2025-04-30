// motion_detector.c
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
        // Exponentially Weighted Moving Average
           // https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
uint16_t SoundData;         // raw data sampled from the microphone
int32_t SoundAvg;

uint8_t But2=1;
uint8_t But1=1;
uint8_t Saw=0;
uint8_t Heard=0;
uint16_t ArmCount = 0;

// semaphores
int32_t NewData;  // true when new numbers to display on top of LCD
int32_t LCDmutex; // exclusive access to LCD
int ReDrawAxes = 0;         // non-zero means redraw axes on next display task

enum plotstate{
  Cam,
  Microphone,
};

enum ArmState{
	On,
	Off
};

enum ArmState Armed = Off;
enum ArmState ArmWas = On;
enum plotstate PlotState = Cam;
//color constants
#define BGCOLOR     LCD_BLACK
#define PERSONCOLOR    LCD_YELLOW
#define SOUNDCOLOR  LCD_CYAN
#define HEARDCOLOR 	LCD_RED

#define NOCOLOR LCD_WHITE

//------------ end of Global variables shared between tasks -------------



// Buffer for receiving commands
#define BUFFER_SIZE 32
char rxBuffer[BUFFER_SIZE];
uint8_t rxIndex = 0;

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

// *********Task0 Handles UART Bluetooth Data Retrieval*********

void Task0(void){
  uint16_t DetCount = 0;
	uint16_t ChirpCount = 0;
	
	while(1) {
		
		// Stores data from UART FIFO inot int c
        int c = UART1_ReceiveChar();
        
        if(c != -1) {  // If data received
            // Echo character back for terminal feedback
            UART1_SendChar((uint8_t)c);
            
				// if c is "1", set Saw to 1 and Chip speaker. additional chirps taken in intervals.
					if ( c == '1'){
						DetCount = 0;
						if (PlotState == Cam && (Saw == 0 || ChirpCount >= 1) && Armed == On){
							BSP_Buzzer_Set(512);  
							ChirpCount = 0;
							BSP_Delay1ms(100);
							BSP_Buzzer_Set(0);
						}
	
						Saw = 1;}
					// count to hold detection for about a second.	
					if( DetCount > 3)
						Saw = 0;
					
					DetCount ++;
					ChirpCount ++;

        }
    }
	
  
}

#define SOUNDRMSLENGTH 1000 // number of samples to collect before calculating RMS (may overflow if greater than 4104)
int16_t SoundArray[SOUNDRMSLENGTH];
// *********Task1_Init*********
// initialize Microphone
void Task1_Init(void){
BSP_Microphone_Init();
}
// *********Task1: Takes in shound data and determines if sound detection occurs*********
 #define SOUND_THRESHOLD 517  // Adjused this based on testing with microphone
 void Task1(void){
     static int32_t soundSum = 0;
     static int time = 0; // units of microphone sampling rate
     static int soundCounter = 0;  // For debouncing sound detection
	 uint16_t HDetCount = 0;
     while(1){
         Profile_Toggle0(); 
         BSP_Microphone_Input(&SoundData);
         soundSum = soundSum + (int32_t)SoundData;
         SoundArray[time] = SoundData;
         time = time + 1;
         
         // Sound threshold detection
         if(PlotState == Microphone) {
             // Check if sound is above threshold
             if(SoundData > SOUND_THRESHOLD) {
                 soundCounter++;
                 if(soundCounter > 10) {  // Require multiple samples above threshold
									 
									 HDetCount = 0;
                     Heard = 1;                    
                 }
             } else {
                 soundCounter = 0;
                 if(Heard && HDetCount > 20) {
                     Heard = 0;
                 }
             }
						 if(Heard){
						 HDetCount++;
							 BSP_Delay1ms(27);
						 }						 
         }        
     }
 }


//---------------- Task2 Populates LCD Screen based on global variables ----------------

void Task2(void){
  while(1){

// if both buttons pressed, Arm or disarm the system.
    OS_Wait(&LCDmutex);
		if (But2 == 0 && But1 == 0){
			// button hold time to activate buttons.
			if(ArmCount >= 10){
				// if system is on, disarm, and chirp 3 times
				if(Armed == On)
					Armed = Off;

					BSP_Delay1ms(250);
				
				BSP_Buzzer_Set(512);  
							BSP_Delay1ms(100);
							BSP_Buzzer_Set(0);				
				BSP_Buzzer_Set(512);  
							BSP_Delay1ms(100);
							BSP_Buzzer_Set(0);								
				BSP_Buzzer_Set(512);  
							BSP_Delay1ms(100);
							BSP_Buzzer_Set(0);
			
				ArmCount = 0;
				
				BSP_Delay1ms(250);
		// If system is off, arm and chirp twice.
			}else if (Armed == Off){
					Armed = On;
				
								BSP_Delay1ms(250);

				BSP_Buzzer_Set(512);  
							BSP_Delay1ms(100);
							BSP_Buzzer_Set(0);			
				BSP_Buzzer_Set(512);  
							BSP_Delay1ms(100);
							BSP_Buzzer_Set(0);
				
				BSP_Delay1ms(250);
				
				ArmCount = 0;
			}
				ArmCount++;
			}
		// if no buttons pressed and System is Off, draw white line at bottom of screen and display "Alarm System Disabled"
		else if(Armed == Off){
			BSP_LCD_PlotPoint(30, NOCOLOR);
			if (ArmWas == On){
			BSP_LCD_FillRect(25, 40, 90, 40, LCD_BLACK);
			ArmWas = Off;
			}
			
			BSP_LCD_DrawString(5, 5, "Alarm",  NOCOLOR);
			BSP_LCD_DrawString(5, 6, "System",  NOCOLOR);
			BSP_LCD_DrawString(5, 7, "Disabled",  NOCOLOR);
			
		}
	// if plot state is Cam and either Button 2 is pressed or Saw is 1 indicating movement from the camera, draw an
		// elevated yellow line and display "Person Detected Camera"
    else if ( PlotState == Cam && ( But2 == 0 || Saw ==1)){
		
			if (ArmWas == Off){
			BSP_LCD_FillRect(25, 40, 90, 40, LCD_BLACK);
			ArmWas = On;
			}
			
			BSP_LCD_PlotPoint(150, PERSONCOLOR);
			
      BSP_LCD_DrawString(5, 5, "Person",  PERSONCOLOR);
			BSP_LCD_DrawString(5, 6, "Detected",  PERSONCOLOR);
			BSP_LCD_DrawString(5, 7, "Camera",  PERSONCOLOR);
			
			BSP_Delay1ms(25);
			
		}
		// if plot state is Microphone and either Button 2 is pressed or Heard is 1 indicating noise from the microphone, draw an
		// elevated red line and display "Person Detected Mic"
		else if(PlotState == Microphone && (But2 == 0 || Heard == 1)){
			
			if (ArmWas == Off){
			BSP_LCD_FillRect(25, 40, 90, 40, LCD_BLACK);
			ArmWas = On;
			}
			
      BSP_LCD_PlotPoint(SoundData/10, SOUNDCOLOR);
			
			BSP_LCD_PlotPoint(150, HEARDCOLOR);
			
      BSP_LCD_DrawString(5, 5, "Person",  HEARDCOLOR);
			BSP_LCD_DrawString(5, 6, "Detected",  HEARDCOLOR);
			BSP_LCD_DrawString(5, 7, "Mic",  HEARDCOLOR);
			
			BSP_Delay1ms(25);			
    } 
		//If system is armed but no inputs recived, draw a white line at bottom of screen.
		else if((Saw == 0 && PlotState == Cam) || But2 != 0  ){
      BSP_LCD_PlotPoint(30, NOCOLOR);
			
			BSP_LCD_FillRect(25, 40, 90, 40, LCD_BLACK);			
    } 
    BSP_LCD_PlotIncrement();
    OS_Signal(&LCDmutex);
  }
}

//------------Task3 Monitors Button Inputs.-------

void Task3(void){
  static uint8_t prev1 = 0, prev2 = 0;

  BSP_Button1_Init();
  BSP_Button2_Init();
  BSP_Buzzer_Init(0);
  BSP_RGB_Init(0, 0, 0);
  while(1){
    Profile_Toggle3(); // viewed by a real logic analyzer to know Task3 started
    BSP_Buzzer_Set(0);
    But2 = BSP_Button2_Input();
    if((But2 == 0) && (prev1 != 0)){

      BSP_Buzzer_Set(512);           // beep until next call of this task
    }
    prev1 = But2;
    But1 = BSP_Button1_Input();
    if((But1 == 0) && (prev2 != 0)){
              // redraw axes on next call of display task
			
						if(PlotState == Cam){
								PlotState = Microphone;
						}else if(PlotState == Microphone){
								PlotState = Cam;}


      BSP_Buzzer_Set(512);           // beep until next call of this task
    }
    prev2 = But1;

    BSP_Delay1ms(5); // very inefficient, but does debounce the switches
  }

}



//*************** Main Function *************************

int main(void){
  OS_Init();
  Profile_Init();  // initialize the 7 hardware profiling pins
  Task1_Init();    // accelerometer init
	
	//LCD Initializaton
  BSP_LCD_Init();
	BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
	BSP_LCD_Drawaxes(PERSONCOLOR, BGCOLOR, "Time", "", 0,"", 0, 500, 0);
	BSP_LCD_DrawString(0,1, "Security Sys",  PERSONCOLOR);
  Time = 0;
  OS_InitSemaphore(&NewData, 0);  // 0 means no data
  OS_InitSemaphore(&LCDmutex, 1); // 1 means free
	OS_FIFO_Init();
	
	//Add threads
  OS_AddThreads(&Task2, &Task3, &Task0,&Task1);

    // Initialize UART1 at 9600 baud (no PLL, using 16MHz clock)
    UART1_Init();

    // Send startup message
    UART1_SendString("\r\nTIVA C Command Processor Ready\r\n");

  OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here

	
  return 0;             // this never executes
}

