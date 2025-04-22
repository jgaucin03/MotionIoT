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
    UART1_IBRD_R = 104;                                  // Integer part (16MHz/16/9600)
    UART1_FBRD_R = 11;                                   // Fractional part
    
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

int main(void) {
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
    
    // Main command processing loop
    while(1) {
        int c = UART1_ReceiveChar();
        
        if(c != -1) {  // If data received
            // Echo character back for terminal feedback
            UART1_SendChar((uint8_t)c);
            
            // Check for carriage return or newline (command terminator)
            if (c == '\r' || c == '\n') {
                if (rxIndex > 0) {
                    // Process the command
                    UART1_SendString("\r\n");  // Add newline for better readability
                    processCommand();
                }
            }
            else if (rxIndex < BUFFER_SIZE - 1) {
                // Add character to buffer
                rxBuffer[rxIndex++] = (char)c;
            }
        }
    }
}