// Start Original
// os.c
// Runs on LM4F120/TM4C123/MSP432
// Lab 2 starter file.
// Daniel Valvano
// February 20, 2016

#include <stdint.h>
#include "os.h"
#include "../inc/CortexM.h"
#include "../inc/BSP.h"

// function definitions in osasm.s
void StartOS(void);

tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];


// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: systick, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
// Jonathan Gaucin: Define global variables void (*PeriodicThread1)(void) = 0;

// #define MAX_PERIODIC_THREADS 2

void (*PeriodicThread1)(void);
void (*PeriodicThread2)(void);
uint32_t Period1;
uint32_t Period2;
uint32_t Count1;
uint32_t Count2;

void OS_Init(void){
  DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
  // initialize any global variables as needed
  //***YOU IMPLEMENT THIS FUNCTION*****
	PeriodicThread1 = 0;
	PeriodicThread2 = 0;
	Period1 = 0;
	Period2 = 0;
	Count1 = 0;
	Count2 = 0;
	EnableInterrupts();
}

void SetInitialStack(int i){
  //***YOU IMPLEMENT THIS FUNCTION*****
	// Jonathan Gaucin: Virtual registers. These are arbitrary memory locations used by each thread as their 'individual R0-R15'.
	tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
	//Stacks[i][STACKSIZE-2] = 0x14141414;   // R15 PC Not included here. Used later for intialization
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

//******** OS_AddThreads ***************
// Add four main threads to the scheduler
// Inputs: function pointers to four void/void main threads
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch

int OS_AddThreads(void(*thread0)(void),
                  void(*thread1)(void),
                  void(*thread2)(void),
                  void(*thread3)(void)){
// initialize TCB circular list
// initialize RunPt
// initialize four stacks, including initial PC
  //***YOU IMPLEMENT THIS FUNCTION*****
	int32_t status;
	status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[3]; // 2 points to 3
	tcbs[3].next = &tcbs[0]; // 3 points to 0
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(thread0); // PC
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(thread1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(thread2); // PC
	SetInitialStack(3); Stacks[3][STACKSIZE-2] = (int32_t)(thread3); // PC
											//STACKSIZE-2 IS PC pg 179
	RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status);

  return 1;               // successful
}

//******** OS_AddThreads3 ***************
// add three foregound threads to the scheduler
// This is needed during debugging and not part of final solution
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added

int OS_AddThreads3(void(*task0)(void),
                 void(*task1)(void),
                 void(*task2)(void)){ 
	// initialize TCB circular list (same as RTOS project)
	// initialize RunPt
	// initialize four stacks, including initial PC
  //***YOU IMPLEMENT THIS FUNCTION*****
	int32_t status2;
  status2 = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[0]; // 2 points to 0
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(task0); // PC initialized for the three threads to start with.
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(task1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(task2); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status2);

  return 1;               // successful
}
                 
//******** OS_AddPeriodicEventThreads ***************
// Add two background periodic event threads
// Typically this function receives the highest priority
// Inputs: pointers to a void/void event thread function2
//         periods given in units of OS_Launch (Lab 2 this will be msec)
// Outputs: 1 if successful, 0 if this thread cannot be added
// It is assumed that the event threads will run to completion and return
// It is assumed the time to run these event threads is short compared to 1 msec
// These threads cannot spin, block, loop, sleep, or kill
// These threads can call OS_Signal
int OS_AddPeriodicEventThreads(void(*thread1)(void), uint32_t period1,
  void(*thread2)(void), uint32_t period2){
  //***YOU IMPLEMENT THIS FUNCTION*****
	//Store the thread functions and their periods
  PeriodicThread1 = thread1;
  PeriodicThread2 = thread2;
  Period1 = period1;
  Period2 = period2;
  Count1 = 0;
  Count2 = 0;
  return 1;
}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
// runs every ms
// LCM Helper functions
int gcd(int a, int b) {
    while (b != 0) {
        int temp = b;
        b = a % b;
        a = temp;
    }
    return a;
}

// Function to compute the Least Common Multiple (LCM)
int lcm(int period1, int period2) {
    return (period1 / gcd(period1, period2)) * period2;
}
uint32_t Counter;
void Scheduler(void){ // every time slice
  // run any periodic event threads if needed
  // implement round robin scheduler, update RunPt
  //***YOU IMPLEMENT THIS FUNCTION*****
	Count1++;
  Count2++;
  
  if(Count1 >= Period1 && Period1 > 0){
    Count1 = 0;
    if(PeriodicThread1 != 0){
      (*PeriodicThread1)();
    }
  }
  
  if(Count2 >= Period2 && Period2 > 0){
    Count2 = 0;
    if(PeriodicThread2 != 0){
      (*PeriodicThread2)();
    }
  }
  
  // Move to next thread in round robin fashion
  RunPt = RunPt->next;
	/*if(Period1>0 && Period2>0){ //run if needed, skipped on startup bc both are 0
        //code for LCM
        uint32_t LeastCommonMultiple=1;
        LeastCommonMultiple=lcm(Period1,Period2);
        Counter = Period1*Period2%LeastCommonMultiple; // LCM as per notes
        if((Counter%Period1) == 1){ 
            Count1=Counter;
            PeriodicThread1();
        }
        if((Counter%(Period2)) == 0){ 
            Count2=Counter;
            PeriodicThread2();
        }
		}
		RunPt = RunPt->next;*/
}

// Jonathan Gaucin Notes: 
// The function main_step1 needs you to have a functioning spin-lock Semaphore system. 
// This allows different threads to inform one another that they are using a certain function that can only be
// used one at a time, like how only one program can write to the lcd at a time. 
// You will have to fill in these functions in the os.c file.

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
  //***YOU IMPLEMENT THIS FUNCTION*****\
	// Jonathan Gaucin: Implementation of a counting spinlock semaphore. Atomic task, disable interrupts: Read-modify-write counter.
	// Jonathan Gaucin: Initialization, assign the given value to the semaphore integer. 
	// Jonathan Gaucin: Account for negative numbers (lowest number should be 0.). TODO: Lab 3 requires negative numbers (There is a reason!).
	DisableInterrupts();
	if (*semaPt < 0) {
		*semaPt = 0;
	}
	else *semaPt = value;
	EnableInterrupts();
	return;
}

// ******** OS_Wait ************
// Decrement semaphore
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
	// Jonathan Gaucin: Decrement the semaphore integer by 1, cannot go below 0. Atomic task, disable interrupts during execution: Read-modify-write counter.
	DisableInterrupts();
	// Jonathan Gaucin: Spin-lock semaphore. If state is busy (0), semaphore 'spins' until another thread/{interrupt} signals it is free (1) or scheduler timesout.
	// Jonathan Gaucin: Check if semaphore is busy (0), if so, enable interrupts:
	// Jonathan Gaucin: Loop to allow SysTick timeout instead of letting software hang. Or allow for seperate process to increment counter.

		EnableInterrupts();
		DisableInterrupts();

	// Jonathan Gaucin: Case semaphore greater than 0, initially or via interrupt. Enable interrupts at end.
	*semaPt = *semaPt - 1;
	EnableInterrupts();
	return;
}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
//***YOU IMPLEMENT THIS FUNCTION*****
	// Jonathan Gaucin: Increment the semaphore integer by 1, all instances. Atomic task, disable interrupts: Read-modify-write counter.
	DisableInterrupts();
	*semaPt = *semaPt + 1;
	EnableInterrupts();
	return;
}




// ******** OS_MailBox_Init ************
// Initialize communication channel
// Producer is an event thread, consumer is a main thread
// Inputs:  none
// Outputs: none
// Jonathan Gaucin: Implement three global variables: Mail (Data) Send, Ack (Semaphores)
uint32_t Mail; // shared data
int32_t Send = 0; // semaphore
int32_t Ack=0; // semaphore
uint32_t Lost=0; // Jonathan Gaucin: Optional Lost variable to account for lost 
void OS_MailBox_Init(void){
  // include data field and semaphore
  //***YOU IMPLEMENT THIS FUNCTION*****
	// Jonathan Gaucin: Initialize MailBox-associated global variables. 
	// Jonathan Gaucin: Mail: (Data) 0 and Semaphores: (Send), (Ack) 0.
	// Initialize data: Mail = 0;
	// Initialize semaphore: Ack = 0;
	// Initialize semaphore: Send = 0;
	
	Mail = 0;
	OS_InitSemaphore(&Ack, 0); // Jonathan Gaucin: Main thread (Consumer)
	OS_InitSemaphore(&Send, 0); // Jonathan Gaucin: Event thread (Producer)
	Lost = 0; // Jonathan Gaucin: Optional error counter (mailbox messages lost)
	
	return;
}

// ******** OS_MailBox_Send ************
// Enter data into the MailBox, do not spin/block if full
// Use semaphore to synchronize with OS_MailBox_Recv
// Inputs:  data to be sent
// Outputs: none
// Errors: data lost if MailBox already has data
void OS_MailBox_Send(uint32_t data){
  //***YOU IMPLEMENT THIS FUNCTION*****
	// Jonathan Gaucin: Store data (Mail) and set semaphore signal (Send).
	// Jonathan Gaucin: Producer is an event thread (Sends data). Cannot call OS_Wait() (not a main thread).
	// Jonathan Gaucin: Mailbox is lock-step. Error occurs if Send is 1 and then Send is called again: [Send, Send (data loss error), ...].
	// Jonathan Gaucin: Lock-step implies [send, receive, send, ...] or [receive, send, receive, ...].
	// Jonathan Gaucin: This implementation destroys old data in case of error. Mail (Data) = data;
	
	Mail = data;
	
	if (Send) {
		Lost++; // Increment error counter if Send is called while initially 1.
	}
	else {
		OS_Signal(&Send); // Jonathan Gaucin: Increment Semaphore
	}
	
	return;
}

// ******** OS_MailBox_Recv ************
// retreive mail from the MailBox
// Use semaphore to synchronize with OS_MailBox_Send
// Lab 2 spin on semaphore if mailbox empty
// Lab 3 block on semaphore if mailbox empty
// Inputs:  none
// Outputs: data retreived
// Errors:  none
uint32_t OS_MailBox_Recv(void){ uint32_t data;
  //***YOU IMPLEMENT THIS FUNCTION*****
	// Jonathan Gaucin: Consumer is a main thread, can call OS_Wait().
	// Jonathan Gaucin: If MailBox empty, wait for send signal. If MailBox full, read data, signal read, return data.
	// Jonathan Gaucin: Accounts for Send = 0 and Send = 1 case with spin-lock semaphore, returning once Send signal is (or became) 1.
	
	OS_Wait(&Send); // Jonathan Gaucin: Wait for Send signal to become 1 (if not already), which then gets decremented to 0 then returns here.
	data = Mail; // Jonathan Gaucin: Transfer MailBox data into local variable to return it later.

	OS_Signal(&Ack); // Jonathan Gaucin: Increment Ack semaphore, data received.
	
  return data;
}
// End Original



#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  
// One event thread producer, one main thread consumer
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
//***IMPLEMENT THIS***
	PutI = GetI = 0; // Jonathan Gaucin: Initialize FIFO to be empty.
	OS_InitSemaphore(&CurrentSize, 0); // Jonathan Gaucin: Initialize Semaphore to 0.
	LostData = 0; // Jonathan Gaucin: Initially 0.
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  
// Exactly one event thread puts,
// do not block or spin if full
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
//***IMPLEMENT THIS***
	
	if (CurrentSize == FSIZE) {
		LostData++;
		return -1; // Full
	}
	else {
		Fifo[PutI] = data; // Put
		PutI = (PutI+1)%FSIZE;
		OS_Signal(&CurrentSize);
		return 0; // success
	}

}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.   
// Exactly one main thread get,
// do block if empty
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
//***IMPLEMENT THIS***
	OS_Wait(&CurrentSize); // Block if empty
	data = Fifo[GetI]; // get
	GetI = (GetI+1)%FSIZE; // Place to get next
  return data;
}
