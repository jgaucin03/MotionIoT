// Motion_Detection_RTOS.c
// Security system with Bluetooth communication using RTOS concepts
// Based on Lab2.c by Daniel and Jonathan Valvano
// Modified by Team 12: Luke Dvorak, Jonathan Gaucin, and Jonathan McChesney-Fleming
// April 22, 2025

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../inc/BSP.h"
#include "../inc/Profile.h"
#include "../inc/CortexM.h"
#include "../inc/tm4c123gh6pm.h"
#include "os.h"

uint32_t sqrt32(uint32_t s);
#define THREADFREQ 1000   // frequency in Hz of round robin scheduler

//---------------- Global variables shared between tasks ----------------
uint32_t Time;              // elapsed time in 100 ms units
uint32_t Steps;             // number of steps counted
uint32_t Magnitude;         // will not overflow (3*1,023^2 = 3,139,587)
uint32_t EWMA;              // Exponentially Weighted Moving Average
uint16_t SoundData;         // raw data sampled from the microphone
int32_t SoundAvg;           // average sound level
uint32_t LightData;         // light sensor data
int32_t TemperatureData;    // temperature in 0.1C

uint8_t PersonDetected = 0; // Flag indicating if a person is detected
uint8_t MotionSource = 0;   // Source of the motion detection (1=button, 2=accel, 3=sound)

// Buffer for receiving commands via UART
#define BUFFER_SIZE 32
char rxBuffer[BUFFER_SIZE];
uint8_t rxIndex = 0;

// semaphores
int32_t NewData;            // true when new numbers to display on top of LCD
int32_t LCDmutex;           // exclusive access to LCD
int32_t UARTmutex;          // exclusive access to UART
int32_t BluetoothRxSema;    // Bluetooth data received
int32_t PersonDetectSema;   // person detection semaphore
int ReDrawAxes = 0;         // non-zero means redraw axes on next display task

// Thresholds for detection
#define ACCEL_THRESHOLD 1125    // Threshold for accelerometer magnitude to detect motion
#define SOUND_THRESHOLD 1    // Threshold for sound level to detect motion
#define DETECTION_TIMEOUT 50   // Number of cycles to keep detection active (5 seconds @ 10Hz)

// Plot states
enum plotstate{
  Accelerometer,
  Microphone
};
enum plotstate PlotState = Accelerometer;

// Color constants
#define BGCOLOR     LCD_BLACK
#define AXISCOLOR   LCD_ORANGE
#define MAGCOLOR    LCD_YELLOW
#define PERSONCOLOR LCD_YELLOW
#define EWMACOLOR   LCD_CYAN
#define SOUNDCOLOR  LCD_CYAN
#define TEMPCOLOR   LCD_LIGHTGREEN
#define TOPTXTCOLOR LCD_WHITE
#define TOPNUMCOLOR LCD_ORANGE

// Forward declarations to fix errors
void UART1_SendString(char *str);
int OS_PeriodCount(int32_t *semaPt, int32_t expectValue);

//---------------- Task0 samples sound from microphone ----------------
// Event thread run by OS in real time at 1000 Hz
#define SOUNDRMSLENGTH 1000 // number of samples to collect before calculating RMS
int16_t SoundArray[SOUNDRMSLENGTH];

// *********Task0_Init*********
// initializes microphone
// Task0 measures sound intensity
// Inputs:  none
// Outputs: none
void Task0_Init(void){
  BSP_Microphone_Init();
}

// *********Task0*********
// Periodic event thread runs in real time at 1000 Hz
// collects data from microphone
// Inputs:  none
// Outputs: none
void Task0(void){
  static int32_t soundSum = 0;
  static int time = 0; // units of microphone sampling rate

  // record system time in array, toggle virtual logic analyzer
  Profile_Toggle0(); // viewed by a real logic analyzer to know Task0 started
  
  // ADC is shared, but on the TM4C123 it is not critical with other ADC inputs
  BSP_Microphone_Input(&SoundData);
  soundSum = soundSum + (int32_t)SoundData;
  SoundArray[time] = SoundData;
  time = time + 1;
  
  if(time == SOUNDRMSLENGTH){
    SoundAvg = soundSum/SOUNDRMSLENGTH;
    soundSum = 0;
    
    // Check if sound level exceeds threshold for person detection
    if(SoundAvg > SOUND_THRESHOLD) {
      PersonDetected = 1;
      MotionSource = 3; // Sound detection
      OS_Signal(&PersonDetectSema);
    }
    
    OS_Signal(&NewData); // makes task5 run every 1 sec
    time = 0;
  }
}

//---------------- Task1 measures acceleration ----------------
// Event thread run by OS in real time at 10 Hz
uint16_t AccX, AccY, AccZ;  // returned by BSP as 10-bit numbers

// *********Task1_Init*********
// initializes accelerometer
// Task1 counts Steps
// Inputs:  none
// Outputs: none
void Task1_Init(void){
  BSP_Accelerometer_Init();
  // initialize the exponential weighted moving average filter
  BSP_Accelerometer_Input(&AccX, &AccY, &AccZ);
  Magnitude = sqrt32(AccX*AccX + AccY*AccY + AccZ*AccZ);
  EWMA = Magnitude;                // this is a guess; there are many options
  Steps = 0;
}

// *********Task1*********
// collects data from accelerometer
// sends data to Task2
// Inputs:  none
// Outputs: none
void Task1(void){
  uint32_t squared;
  
  // records system time in array, toggles virtual logic analyzer
  Profile_Toggle1(); // viewed by a real logic analyzer to know Task1 started

  BSP_Accelerometer_Input(&AccX, &AccY, &AccZ);
  squared = AccX*AccX + AccY*AccY + AccZ*AccZ;
  Magnitude = sqrt32(squared);
  
  // Check for sudden movement
  if(Magnitude > ACCEL_THRESHOLD) {
    PersonDetected = 1;
    MotionSource = 2; // Accelerometer detection
    OS_Signal(&PersonDetectSema);
  }
  
  OS_MailBox_Send(squared); // makes Task2 run every 100ms
  Time++; // in 100ms units
}

//---------------- Task2 process data and plots on LCD ----------------
// Main thread scheduled by OS round robin preemptive scheduler
void Task2(void){
  uint32_t data;
  static uint8_t detectionCounter = 0;
  
  // Initial display
  OS_Wait(&LCDmutex);
  BSP_LCD_DrawString(5, 2, "Security System", TOPTXTCOLOR);
  BSP_LCD_DrawString(5, 3, "Status: Ready", PERSONCOLOR);
  BSP_LCD_Drawaxes(PERSONCOLOR, BGCOLOR, "Time", "", 0, "", 0, 500, 0);
  OS_Signal(&LCDmutex);

  while(1){
    data = OS_MailBox_Recv(); // acceleration data from Task 1
    
    // records system time in array, toggles virtual logic analyzer
    Profile_Toggle2(); // viewed by a real logic analyzer to know Task2 started
    
    // If detection active, decrement counter
    if(detectionCounter > 0) {
      detectionCounter--;
      if(detectionCounter == 0) {
        PersonDetected = 0; // Clear detection after timeout
        
        OS_Wait(&LCDmutex);
        BSP_LCD_FillRect(0, 25, 127, 20, BGCOLOR); // Clear detection message
        BSP_LCD_DrawString(5, 3, "Status: Ready     ", PERSONCOLOR);
        OS_Signal(&LCDmutex);
        
        // Send LED command via UART
        OS_Wait(&UARTmutex);
        UART1_SendString("LED:G\r\n");
        OS_Signal(&UARTmutex);
      }
    }
    
    if(ReDrawAxes){
      OS_Wait(&LCDmutex);
      BSP_LCD_Drawaxes(PERSONCOLOR, BGCOLOR, "Time", "", 0, "", 0, 500, 0);
      OS_Signal(&LCDmutex);
      ReDrawAxes = 0;
    }
    
    OS_Wait(&LCDmutex);
    // Plot points based on current state - only Accelerometer and Microphone
    if(PlotState == Accelerometer){
      BSP_LCD_PlotPoint(Magnitude/10, MAGCOLOR); // Scale down for display
    } else if(PlotState == Microphone){
      BSP_LCD_PlotPoint(SoundData, SOUNDCOLOR);
    }
    BSP_LCD_PlotIncrement();
    OS_Signal(&LCDmutex);
    
    if(OS_PeriodCount(&PersonDetectSema, 0)) { // Non-blocking check
      detectionCounter = DETECTION_TIMEOUT; // Reset detection timeout
      
      OS_Wait(&LCDmutex);
      
      // Display detection message based on source
      BSP_LCD_FillRect(0, 25, 127, 20, BGCOLOR); // Clear previous message
      BSP_LCD_DrawString(5, 3, "Status: ALERT!    ", LCD_RED);
      
      switch(MotionSource) {
        case 1:
          BSP_LCD_DrawString(5, 5, "Person Detected!", PERSONCOLOR);
          BSP_LCD_DrawString(5, 6, "Source: Button", PERSONCOLOR);
          break;
        case 2:
          BSP_LCD_DrawString(5, 5, "Person Detected!", PERSONCOLOR);
          BSP_LCD_DrawString(5, 6, "Source: Motion", PERSONCOLOR);
          break;
        case 3:
          BSP_LCD_DrawString(5, 5, "Person Detected!", PERSONCOLOR);
          BSP_LCD_DrawString(5, 6, "Source: Sound", PERSONCOLOR);
          break;
        case 4:
          BSP_LCD_DrawString(5, 5, "Person Detected!", PERSONCOLOR);
          BSP_LCD_DrawString(5, 6, "Source: Camera", PERSONCOLOR);
          break;
        default:
          BSP_LCD_DrawString(5, 5, "Person Detected!", PERSONCOLOR);
          BSP_LCD_DrawString(5, 6, "Source: Unknown", PERSONCOLOR);
      }
      OS_Signal(&LCDmutex);
      
      // Send LED command via UART
      OS_Wait(&UARTmutex);
      UART1_SendString("LED:R\r\n");
      OS_Signal(&UARTmutex);
    }
  }
}

//------------Task3 handles switch input, buzzer output, LED output-------
// *********Task3*********
// Main thread scheduled by OS round robin preemptive scheduler
// non-real-time task
// checks the switches, updates the mode, and outputs to the buzzer 
// Inputs:  none
// Outputs: none
// Updated Task3 function to only toggle between Accelerometer and Microphone
void Task3(void){
  static uint8_t prev1 = 0, prev2 = 0;
  uint8_t current;

  BSP_Button1_Init();
  BSP_Button2_Init();
  BSP_Buzzer_Init(0);
  BSP_RGB_Init(0, 0, 0);
  
  while(1){
    Profile_Toggle3(); // viewed by a real logic analyzer to know Task3 started
    
    BSP_Buzzer_Set(0);
    current = BSP_Button1_Input();
    
    if((current == 0) && (prev1 != 0)){
      // Button1 was pressed since last loop
      PersonDetected = 1;
      MotionSource = 1; // Button detection
      OS_Signal(&PersonDetectSema);
      
      // Only toggle between Accelerometer and Microphone
      if(PlotState == Accelerometer){
        PlotState = Microphone;
      } else {
        PlotState = Accelerometer;
      }
      
      ReDrawAxes = 1;
      BSP_Buzzer_Set(512); // beep
    }
    prev1 = current;
    
    current = BSP_Button2_Input();
    if((current == 0) && (prev2 != 0)){
      // Button2 was pressed since last loop
      // Also just toggle between Accelerometer and Microphone
      if(PlotState == Accelerometer){
        PlotState = Microphone;
      } else {
        PlotState = Accelerometer;
      }
      
      ReDrawAxes = 1;
      BSP_Buzzer_Set(512); // beep
    }
    prev2 = current;

    BSP_Delay1ms(5); // debounce switches
  }
}

// Initialize UART1 for communication with HC-05
void InitUART1(void){
  // Enable UART1 and PORTB clocks
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
  
  // Wait for the clocks to stabilize
  while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R1) == 0){}
  
  // Configure GPIO pins for UART
  GPIO_PORTB_AFSEL_R |= 0x03;                          // Enable alternate function
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) | 0x00000011; // Configure for UART
  GPIO_PORTB_DEN_R |= 0x03;                            // Enable digital I/O
  
  // Configure UART1
  UART1_CTL_R &= ~UART_CTL_UARTEN;                     // Disable UART for setup
  
  // Configure for 9600 baud rate
  // Assumes 16MHz system clock (no PLL)
  UART1_IBRD_R = 104;                                  // Integer part (16MHz/16/9600)
  UART1_FBRD_R = 11;                                   // Fractional part
  
  // 8-bit data, 1 stop bit, no parity
  UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;     // 8-bit, FIFOs enabled
  
  // Enable interrupts
  UART1_IM_R |= UART_IM_RXIM;                          // Enable RX interrupt
  NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF00FFFF) | 0x00400000; // Priority 2
  NVIC_EN0_R = 0x00000040;                            // Enable interrupt 6 in NVIC
  
  UART1_CTL_R |= UART_CTL_UARTEN;                      // Enable UART
}

// UART1 Interrupt Handler
void UART1_Handler(void){
  char data;
  
  // Check if interrupt is due to received data
  if(UART1_RIS_R & UART_RIS_RXRIS){
    // Clear interrupt flag
    UART1_ICR_R = UART_ICR_RXIC;
    
    // Read data
    data = (char)(UART1_DR_R & 0xFF);
    
    // Check for carriage return or newline (command terminator)
    if(data == '\r' || data == '\n'){
      if(rxIndex > 0){
        rxBuffer[rxIndex] = '\0';  // Null terminate
        OS_Signal(&BluetoothRxSema);  // Signal command ready
        rxIndex = 0;               // Reset buffer index
      }
    }
    else if(rxIndex < BUFFER_SIZE - 1){
      // Add character to buffer
      rxBuffer[rxIndex++] = data;
    }
  }
}

// Send a character via UART1
void UART1_SendChar(uint8_t data){
  while((UART1_FR_R & UART_FR_TXFF) != 0){} // Wait until FIFO not full
  UART1_DR_R = data;                       // Send data
}

// Send a string via UART1
void UART1_SendString(char *str){
  while(*str){
    UART1_SendChar((uint8_t)*str);
    str++;
  }
}

// Process UART command
void ProcessCommand(char* cmd){
  if(strcmp(cmd, "PING") == 0){
    // Echo back "PONG\r\n"
    UART1_SendString("PONG\r\n");
  }
  else if(strcmp(cmd, "STATUS") == 0){
    // Send status information
    UART1_SendString("TIVA C Status: OK\r\n");
    UART1_SendString("Security: ");
    if(PersonDetected){
      UART1_SendString("ALERT - Motion Detected!\r\n");
    } else {
      UART1_SendString("Normal\r\n");
    }
    UART1_SendString("System: Ready\r\n");
  }
  else if(strncmp(cmd, "LED:", 4) == 0){
    // LED control command - we keep this to maintain compatibility with existing code
    char color = cmd[4];
    
    switch(color){
      case 'R':
        BSP_RGB_Set(10, 0, 0);
        UART1_SendString("LED set to Red\r\n");
        break;
      case 'G':
        BSP_RGB_Set(0, 10, 0);
        UART1_SendString("LED set to Green\r\n");
        break;
      case 'B':
        BSP_RGB_Set(0, 0, 10);
        UART1_SendString("LED set to Blue\r\n");
        break;
      default:
        if(strncmp(cmd+4, "OFF", 3) == 0){
          BSP_RGB_Set(0, 0, 0);
          UART1_SendString("LED turned off\r\n");
        } else {
          UART1_SendString("Unknown LED color\r\n");
        }
        break;
    }
  }
  else if(strncmp(cmd, "PERSON:", 7) == 0){
    // Remote person detection command (from Python script)
    if(cmd[7] == '1'){
      PersonDetected = 1;
      MotionSource = 4; // Remote detection
      OS_Signal(&PersonDetectSema);
      UART1_SendString("Person detection activated\r\n");
    } else {
      PersonDetected = 0;
      UART1_SendString("Person detection deactivated\r\n");
    }
  }
  else {
    // Unknown command
    UART1_SendString("Unknown command: ");
    UART1_SendString(cmd);
    UART1_SendString("\r\n");
  }
}

// Task4 - Handle Bluetooth communication
void Task4(void){
  char localBuffer[BUFFER_SIZE];
  
  // Initialize UART1 for HC-05 Bluetooth module
  InitUART1();
  
  // Send startup message
  OS_Wait(&UARTmutex);
  UART1_SendString("\r\nSecurity System RTOS Ready\r\n");
  OS_Signal(&UARTmutex);
  
  while(1){
    // Wait for a command to be received
    OS_Wait(&BluetoothRxSema);
    
    // Copy buffer to local storage to avoid race conditions
    strcpy(localBuffer, rxBuffer);
    
    // Process the command
    OS_Wait(&UARTmutex);
    ProcessCommand(localBuffer);
    OS_Signal(&UARTmutex);
  }
}

// Helper function to check semaphore without blocking
int OS_PeriodCount(int32_t *semaPt, int32_t expectValue){
  int result;
  DisableInterrupts();
  if(*semaPt == expectValue){
    result = 0; // semaphore not available
  } else {
    (*semaPt)--;
    result = 1; // semaphore was available, count decremented
  }
  EnableInterrupts();
  return result;
}

// Required function for calculating magnitude
uint32_t sqrt32(uint32_t s){
  // Fast integer square root
  // Algorithm from http://www.codecodex.com/wiki/Calculate_an_integer_square_root
  uint32_t t, q, b, r;
  r = 0;
  b = 0x40000000;
  while(b > s) b >>= 2;
  while(b){
    t = r + b;
    q = s;
    r >>= 1;
    if(q >= t){
      s -= t;
      r += b;
    }
    b >>= 2;
  }
  return r;
}

// Dummy tasks to fill out OS_AddThreads requirement
void Task5(void){  // Dummy task that also handles UART processing
  // Initialize UART1 for HC-05 Bluetooth module
  InitUART1();
  
  // Send startup message
  OS_Wait(&UARTmutex);
  UART1_SendString("\r\nSecurity System RTOS Ready\r\n");
  OS_Signal(&UARTmutex);
  
  char localBuffer[BUFFER_SIZE];
  
  while(1){
    // Check if there's a Bluetooth command waiting
    DisableInterrupts();
    int hasCommand = (BluetoothRxSema > 0);
    EnableInterrupts();
    
    if(hasCommand) {
      OS_Wait(&BluetoothRxSema);
      
      // Copy buffer to local storage to avoid race conditions
      strcpy(localBuffer, rxBuffer);
      
      // Process the command
      OS_Wait(&UARTmutex);
      ProcessCommand(localBuffer);
      OS_Signal(&UARTmutex);
    }
    
    // Give time for other threads - use a small delay
    BSP_Delay1ms(1);
  }
}

void Task6(void){  // Dummy task that acts as a system monitor
  uint32_t counter = 0;
  
  while(1){
    counter++;
    
    // Every ~5 seconds, show system status on console
    if(counter >= 500) {
      counter = 0;
      
      OS_Wait(&UARTmutex);
      UART1_SendString("System Status: Running\r\n");
      
      // Show the current detection state
      if(PersonDetected) {
        UART1_SendString("Detection: ACTIVE\r\n");
      } else {
        UART1_SendString("Detection: Inactive\r\n");
      }
      OS_Signal(&UARTmutex);
    }
    
    // Give time for other threads
    BSP_Delay1ms(10);
  }
}

// This is a modified version of main to work with your OS implementation
int main(void){
  OS_Init();
  Profile_Init();  // initialize the 7 hardware profiling pins
  
  // Initialize peripherals
  Task0_Init();    // microphone init
  Task1_Init();    // accelerometer init
  BSP_LCD_Init();
  BSP_LCD_FillScreen(BSP_LCD_Color565(0, 0, 0));
  
  // Initialize semaphores
  OS_InitSemaphore(&NewData, 0);         // 0 means no data
  OS_InitSemaphore(&LCDmutex, 1);        // 1 means free
  OS_InitSemaphore(&UARTmutex, 1);       // 1 means free
  OS_InitSemaphore(&BluetoothRxSema, 0); // 0 means no data
  OS_InitSemaphore(&PersonDetectSema, 0); // 0 means no detection
  
  // Initialize mailbox
  OS_MailBox_Init();
  
  // Set initial time
  Time = 0;
  
  // Use OS_AddThreads which is available in your OS implementation
  // Must provide 4 threads to match the function signature
  OS_AddThreads(&Task2, &Task3, &Task5, &Task6);
  
  // Setup periodic threads
  OS_AddPeriodicEventThreads(&Task0, 1, &Task1, 100);
  
  // Launch RTOS
  OS_Launch(BSP_Clock_GetFreq()/THREADFREQ); // doesn't return, interrupts enabled in here
  
  return 0; // This never executes
}
