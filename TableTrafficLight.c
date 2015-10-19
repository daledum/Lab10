// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// November 7, 2013

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ***** 3. Subroutines Section *****

/* already implemented by the Texas Grader
void PLL_Init(void){

  // 0) Use RCC2

  SYSCTL_RCC2_R |=  0x80000000;  // USERCC2

  // 1) bypass PLL while initializing

  SYSCTL_RCC2_R |=  0x00000800;  // BYPASS2, PLL bypass

  // 2) select the crystal value and oscillator source

  SYSCTL_RCC_R = (SYSCTL_RCC_R &~0x000007C0)   // clear XTAL field, bits 10-6

                 + 0x00000540;   // 10101, configure for 16 MHz crystal

  SYSCTL_RCC2_R &= ~0x00000070;  // configure for main oscillator source

  // 3) activate PLL by clearing PWRDN

  SYSCTL_RCC2_R &= ~0x00002000;

  // 4) set the desired system divider

  SYSCTL_RCC2_R |= 0x40000000;   // use 400 MHz PLL

  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~ 0x1FC00000)  // clear system clock divider

                  + (4<<22);      // configure for 80 MHz clock

  // 5) wait for the PLL to lock by polling PLLLRIS

  while((SYSCTL_RIS_R&0x00000040)==0){};  // wait for PLLRIS bit

  // 6) enable use of PLL by clearing BYPASS

  SYSCTL_RCC2_R &= ~0x00000800;

}
*/

void SysTick_Init(void){

  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup

  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock

}

// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)

void SysTick_Wait(unsigned long delay){

  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait

  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears

  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag

  }

}

// 800000*12.5ns equals 10ms

void SysTick_Wait10ms(unsigned long delay){

  unsigned long i;

  for(i=0; i<delay; i++){

    SysTick_Wait(800000);  // wait 10ms

  }

}

#define SENSOR  (*((volatile unsigned long *)0x4002401C)) // PE2-0 are inputs, PE2 for pedestrians, PE1 for South, PE0 for West
#define GPIO_PORTE_IN  (*((volatile unsigned long *)0x4002401C))

#define LIGHT_CAR   (*((volatile unsigned long *)0x400050FC)) //output 6 leds, PB5-0
#define GPIO_PORTB_OUT  (*((volatile unsigned long *)0x400050FC))

#define LIGHT_PED   (*((volatile unsigned long *)0x40025028))  //output 2 leds, PF3 green for pedestrians si PF1 red for pedestrians 
#define GPIO_PORTF_OUT  (*((volatile unsigned long *)0x40025028)) //took GPIO_PORTF_DATA_BITS_R, with 2 empty bits on the right

struct State {
  unsigned long Out_car; 
	//here we have 0 0, 0 1, 1 0, 1 1, in decimal 0,1,2,3 or, in hex 0x01, 0x02, 0x03, 0x04
	//PF3 is 0x08 and PF1 is 0x02. All 0 is 0x00, All 1 is 0x0A
	unsigned long Out_ped; 
  unsigned long Time;  
  unsigned long Next[8];}; 
typedef const struct State STyp;

//here we have 8, from 0 to 7, in the logical order of occurrence
#define goS   0 // green South 
#define waitS 1 // yellow South
#define goW   2 // green West
#define waitW 3 //yellow West
#define walkYes 4 //green light on pedestrians
#define walkNo 5 //red pedestrianss = don't walk
#define walkOff 6 //nothing on pedestrians
#define flashWalkNo 7 //short don't walk display on red light of pedestrians
#define flashWalkOff 8 //nothing on pedestrians, but shorter	

//here we have 200, 100, 200, 100, 100, 100, 50, 50
STyp FSM[9]=
{
 {0x21, 0x02, 200, {waitS,waitS,goS,waitS,waitS,waitS,waitS,waitS}}, 
 
 {0x22, 0x02, 100, {goW,goW,goS,goW,walkOff,goW,walkYes,goW}},
 
 {0x0C, 0x02, 200, {waitW,goW,waitW,waitW,waitW,waitW,waitW,waitW}},

 {0x14, 0x02, 100, {goS,goW,goS,goS,walkOff,walkYes,goS,walkYes}},
 
 {0x24, 0x08, 200, {flashWalkNo,flashWalkNo,flashWalkNo,flashWalkNo,walkYes,flashWalkNo,flashWalkNo,walkOff}},
 
 {0x24, 0x02, 100, {walkOff,walkOff,walkOff,walkOff,walkOff,walkOff,walkOff,goS}},
 
 {0x24, 0x00, 100, {goS,goW,goS,goW,walkYes,goW,goS,flashWalkNo}},
 
 {0x24, 0x02, 50, {flashWalkOff,flashWalkOff,flashWalkOff,flashWalkOff,flashWalkOff,flashWalkOff,flashWalkOff,flashWalkOff}},
 
 {0x24, 0x00, 50, {walkNo,walkNo,walkNo,walkNo,walkYes,walkNo,walkNo,walkNo}}

};
unsigned long S;  // index to the current state 
unsigned long Input; 

int main(void){ 
  volatile unsigned long delay;
	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate grader and set system clock to 80 MHz
	//PLL_Init();       // set system clock to 80 MHz
  SysTick_Init();
  SYSCTL_RCGC2_R |= 0x32; // Ports B E F
	delay = SYSCTL_RCGC2_R;      
  
// Port E init (input from 3 external switches)
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
	GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
	
	
// Port B init (output to 6 external LEDs)
// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0


//3 Port F init (output to 2 internal LEDs)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)
/*
#define PF3                     (*((volatile unsigned long *)0x40025020))
#define PF1                     (*((volatile unsigned long *)0x40025008))
*/
//unfriendly code here
   GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F - only  for PF0
   GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 si PF1
	 GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF //for PF3 and PF1: 0x0A
   GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0 //for PF3 si PF1: |=0x0A
	 GPIO_PORTF_DIR_R = 0x0A;          // 5) PF3 and PF1 out
	 GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0 // &=~0x0A for PF3 and PF1
	 GPIO_PORTF_DEN_R = 0x0A;          // 7) enable digital I/O on PF4-0 // |=0x0A for PF3 and PF1
  
  EnableInterrupts();
  S = goS; 
	while(1){
    LIGHT_CAR = FSM[S].Out_car;  // set lights for cars
		LIGHT_PED = FSM[S].Out_ped;  // set lights for pedestrians
    SysTick_Wait10ms(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input];  
  }
}

