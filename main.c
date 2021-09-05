#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "SysTickInts.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

//*************************GPIO PORTS***************************//
void PortA_Init(void);
void PortB_Init(void);
void PortC_Init(void);
void PortE_Init(void);
void EdgecounterEW_Init(void);    // PortF
void EdgecounterEW_Init2(void);   // PortD

//*************************Functions***************************//
void NS_ON_EW_OFF(void);
void NS_OFF_EW_ON(void);
void wait(int value);

//*************************Interrupts***************************//
void disable_interrupts(void);
void enable_interrupts(void);
void wait_for_interrupts(void);
void IntDefaultHandler1(void);
void IntDefaultHandler2(void);
void ab(void);

//*************************Variables***************************//
volatile int counterNS=0;
volatile int counterEW =0;
volatile int pass_count = 0;

//*************************MAIN FUNCTION***************************//
int main(void){
    PLL_Init();                 // bus clock at 80 MHz
    EdgecounterEW_Init(); // initialize GPIO Port F interrupt
    EdgecounterEW_Init2();
    PortA_Init();
    PortB_Init();
    PortC_Init();
    PortE_Init();

    pass_count = 0;
    GPIO_PORTE_DATA_R =0x08;
    GPIO_PORTB_DATA_R =0x02;
    GPIO_PORTA_DATA_R =~(0x00);
    GPIO_PORTC_DATA_R =0x10;
    while(true){
        NS_ON_EW_OFF();
        wait(10000000);
        NS_OFF_EW_ON();
        wait(5000000);
    }
}

//*************************GPIO PORTS INITIALIZATION FUNCTIONS***************************//

void PortA_Init(void){
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000001;     // 1) E clock
    delay = SYSCTL_RCGC2_R;           // delay
    GPIO_PORTA_CR_R = 0xFF;           // allow changes to PE5,3-0
    GPIO_PORTA_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTA_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
    //GPIO_PORTE_DIR_R |= 0x08;          // 5) PE3 input
    GPIO_PORTA_DIR_R = 0xFF;          // 5) PE2,PE1,PE0 output
    GPIO_PORTA_AFSEL_R = 0x00;        // 6) no alternate function
    // GPIO_PORTE_PUR_R = 0x08;          // enable pullup resistors on PE3
    GPIO_PORTA_DEN_R = 0xFF;          // 7) enable digital pins PE3-PE0
}

void PortB_Init(void){
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000002;     // 1) E clock
    delay = SYSCTL_RCGC2_R;           // delay
    GPIO_PORTB_CR_R = 0x2F;           // allow changes to PE5,3-0
    GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
    //GPIO_PORTE_DIR_R |= 0x08;          // 5) PE3 input
    GPIO_PORTB_DIR_R = 0x0E;          // 5) PE2,PE1,PE0 output
    GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alternate function
    // GPIO_PORTE_PUR_R = 0x08;          // enable pullup resistors on PE3
    GPIO_PORTB_DEN_R = 0x0F;          // 7) enable digital pins PE3-PE0
}

void PortE_Init(void){
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000010;     // 1) E clock
    delay = SYSCTL_RCGC2_R;           // delay
    GPIO_PORTE_CR_R = 0x2F;           // allow changes to PE5,3-0
    GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTE_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
    //GPIO_PORTE_DIR_R |= 0x08;          // 5) PE3 input
    GPIO_PORTE_DIR_R = 0xFF;          // 5) PE2,PE1,PE0 output
    GPIO_PORTE_AFSEL_R = 0x00;        // 6) no alternate function
    //GPIO_PORTE_PUR_R = 0x08;          // enable pullup resistors on PE3
    GPIO_PORTE_DEN_R = 0x0F;          // 7) enable digital pins PE3-PE0
}

void PortC_Init(void){
    volatile unsigned long delay;
    SYSCTL_RCGC2_R |= 0x00000004;     // 1) E clock
    delay = SYSCTL_RCGC2_R;           // delay
    GPIO_PORTC_CR_R = 0xF0;           // allow changes to PE5,3-0
    GPIO_PORTC_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTC_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
    //GPIO_PORTE_DIR_R |= 0x08;          // 5) PE3 input
    GPIO_PORTC_DIR_R = 0xF0;          // 5) PE2,PE1,PE0 output
    GPIO_PORTC_AFSEL_R = 0x00;        // 6) no alternate function
    //GPIO_PORTE_PUR_R = 0x08;          // enable pullup resistors on PE3
    GPIO_PORTC_DEN_R = 0xF0;          // 7) enable digital pins PE3-PE0
}

void EdgecounterEW_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000020;           // activate clock for PortF
    while ((SYSCTL_PRGPIO_R & 0x00000020) == 0)
    {};                          // wait until PortF is ready
    GPIO_PORTF_LOCK_R = 0x4C4F434B;         // unlock GPIO PortF
    GPIO_PORTF_CR_R = 0x1F;                 // allow changes to PF4-0
    GPIO_PORTF_AMSEL_R = 0x00;              // disable analog on PortF
    GPIO_PORTF_PCTL_R = 0x00000000;         // use PF4-0 as GPIO
    GPIO_PORTF_DIR_R = 0x06;                // PF4,PF0 in, PF3-1 out
    GPIO_PORTF_AFSEL_R = 0x00;              // disable alt function on PF
    GPIO_PORTF_PUR_R = 0x11;                // enable pull-up on PF0,PF4
    GPIO_PORTF_DEN_R = 0x1F;                // enable digital I/O on PF4-0

    GPIO_PORTF_IS_R=0x00;
    GPIO_PORTF_IBE_R=0x00;
    GPIO_PORTF_IEV_R =0x00;
    //GPIO_PORTF_IM_R=0xFF;
    NVIC_EN0_R =0x40000000;
    GPIO_PORTF_IM_R=0x11;
    //  GPIO_PORTF_IM_R=0x01;
}

void EdgecounterEW_Init2(void){

    SYSCTL_RCGC2_R |= 0x00000008; // (a) activate clock for port F
    counterNS = 0;             // (b) initialize pass_count and wait for clock
    GPIO_PORTD_DIR_R &= ~0x01;    // (c) make PF4 in (built-in button)
    GPIO_PORTD_AFSEL_R &= ~0x01;  //     disable alt funct on PF4
    GPIO_PORTD_DEN_R |= 0x01;     //     enable digital I/O on PF4
    // GPIO_PORTE_PCTL_R &= ~0x000F0000; //  configure PF4 as GPIO
    GPIO_PORTD_AMSEL_R &= ~0x01;  //    disable analog functionality on PF4
    GPIO_PORTD_PUR_R |= 0x01;     //     enable weak pull-up on PF4
    GPIO_PORTD_IS_R &= ~0x01;     // (d) PF4 is edge-sensitive
    GPIO_PORTD_IBE_R &= ~0x01;    //     PF4 is not both edges
    GPIO_PORTD_IEV_R &= ~0x01;    //     PF4 falling edge event
    GPIO_PORTD_ICR_R = 0x01;      // (e) clear flag4
    GPIO_PORTD_IM_R |= 0x01;      // (f) arm interrupt on PF4
    NVIC_PRI0_R = (NVIC_PRI0_R&0xFF00FFFF)|0xA0000000; // (g) priority 5
    NVIC_EN0_R |= 0x8;      // (h) enable interrupt 30 in NVIC
    enable_interrupts();           // (i) Enable global Interrupt flag (I)
}

//*************************INTERRUPT FUNCTIONS***************************//

/* Disable interrupts by setting the I bit in the PRIMASK system register */
void disable_interrupts(void) {
    __asm("    CPSID  I\n"
            "    BX     LR");
}

/* Enable interrupts by clearing the I bit in the PRIMASK system register */
void enable_interrupts(void) {
    __asm("    CPSIE  I\n"
            "    BX     LR");
}

/* Enter low-power mode while waiting for interrupts */
void wait_for_interrupts(void) {
    __asm("    WFI\n"
            "    BX     LR");
}

/* Interrupt service routine for SysTick Interrupt */
// Executed every 12.5ns*(period)
void SysTick_Handler(void){
    pass_count++;
    if (pass_count >= 7) {
        GPIO_PORTC_DATA_R = 0x20;
    }
    if (pass_count >= 14) {
        GPIO_PORTC_DATA_R = 0x40;
    }
    if (pass_count >= 21) {
        GPIO_PORTA_DATA_R = 0x90;
    }
    if (pass_count >= 28) {
        GPIO_PORTA_DATA_R = 0x80;
    }
    if (pass_count >= 35) {
        GPIO_PORTA_DATA_R = 0x70;
    }
    if (pass_count >= 42) {
        GPIO_PORTA_DATA_R = 0x60;
    }
    if (pass_count >= 49) {
        GPIO_PORTA_DATA_R = 0x50;
    }
    if (pass_count >= 56) {
        GPIO_PORTA_DATA_R = 0x40;
    }
    if (pass_count >= 63) {
        GPIO_PORTA_DATA_R = 0x30;
    }
    if (pass_count >= 70) {
        GPIO_PORTA_DATA_R = 0x20;
    }
    if (pass_count >= 77) {
        GPIO_PORTA_DATA_R = 0x10;
    }
    if (pass_count >= 84) {
        GPIO_PORTA_DATA_R = 0x00;
    }
    if (pass_count >= 91){
        NVIC_ST_CTRL_R = 0;
        pass_count=0;
        GPIO_PORTA_DATA_R = ~(0x00);
        GPIO_PORTC_DATA_R = 0x10;
    };
}

void IntDefaultHandler1(void){
    GPIO_PORTF_ICR_R = 0x11;

    if (GPIO_PORTF_DATA_R == 0x01){
        counterEW = counterEW + 1;
        if(counterEW > 5){
            counterEW = 0;
        }
    }
    if (GPIO_PORTF_DATA_R == 0x10){
        counterNS = counterNS + 1;
        if(counterNS > 5){
            counterNS = 0;
        }
    }
}

void IntDefaultHandler2(void){
    GPIO_PORTD_ICR_R = 0x01;
    ab();
}

void ab(void){
    SysTick_Init(80000000);        // initialize SysTick timer
}

//*************************OTHER FUNCTIONS***************************//

void NS_ON_EW_OFF(void){

    GPIO_PORTE_DATA_R =0x04;
    wait(1000000);
    GPIO_PORTE_DATA_R =0x02;
    wait(500000);
    GPIO_PORTB_DATA_R =0x08;
}
void NS_OFF_EW_ON(void){

    GPIO_PORTB_DATA_R =0x04;
    wait(1000000);
    GPIO_PORTB_DATA_R =0x02;
    wait(500000);
    GPIO_PORTE_DATA_R =0x08;
}

void wait(int value ){
    int i=0;
    //int one_sec=5333333;
    //value*=one_sec;
    while(i<value)
    {i++;};
}
