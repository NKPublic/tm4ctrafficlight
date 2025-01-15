#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../inc/SysTick.h"  // Include SysTick functions

void delay_ms(unsigned int time);
void PortF_Init(void);
void GPIOPortF_Handler(void);

volatile int currentStep = 1; // Tracks the current step in the loop
volatile uint32_t delayTime = 0; // Variable to keep track of the remaining delay time
volatile int switchPressed = 0; // Flag to check if switch was pressed

int main(void) {
    // Enable clocks for Port A and Port B
    SYSCTL_RCGCGPIO_R |= (1 << 0) | (1 << 1);  // Enable clock for GPIOA (bit 0) and GPIOB (bit 1)
    while ((SYSCTL_PRGPIO_R & 0x03) == 0);     // Wait for the clocks to stabilize

    // Configure PA2, PA3, and PA4 as outputs
    GPIO_PORTA_DIR_R |= (1 << 2) | (1 << 3) | (1 << 4);
    GPIO_PORTA_DEN_R |= (1 << 2) | (1 << 3) | (1 << 4);

    // Configure PB5, PB6, and PB7 as outputs
    GPIO_PORTB_DIR_R |= (1 << 5) | (1 << 6) | (1 << 7);
    GPIO_PORTB_DEN_R |= (1 << 5) | (1 << 6) | (1 << 7);

    // Initialize Port F for switches (SW1 and SW2)
    PortF_Init();

    while (1) {
       

        // Perform actions based on currentStep value
        if (currentStep == 1) {
            // Step 1: Turn on PA2 and PB5 for 10 seconds, turn off PB6
						GPIO_PORTA_DATA_R &= ~(1 << 3); // PA3 OFF  
						GPIO_PORTA_DATA_R |= (1 << 2);  // PA2 ON
            GPIO_PORTB_DATA_R |= (1 << 5);  // PB5 ON
            GPIO_PORTB_DATA_R &= ~(1 << 6); // PB6 OFF
            delay_ms(5000);
            currentStep = 2;
        }

        if (currentStep == 2) {
            // Step 2: Turn off PA2, turn on PA3 for 3 seconds
            GPIO_PORTA_DATA_R &= ~(1 << 2); // PA2 OFF
            GPIO_PORTA_DATA_R |= (1 << 3);  // PA3 ON
						currentStep = 3;
            delay_ms(1500);
            
        }

        if (currentStep == 3) {
            // Step 3: Turn off PB5, turn on PA4 and PB7 for 10 seconds, turn off PA3 and PB6
            GPIO_PORTB_DATA_R &= ~(1 << 5); // PB5 OFF
            GPIO_PORTA_DATA_R |= (1 << 4);  // PA4 ON
            GPIO_PORTA_DATA_R &= ~(1 << 3); // PA3 OFF
            GPIO_PORTB_DATA_R |= (1 << 7);  // PB7 ON
            GPIO_PORTB_DATA_R &= ~(1 << 6); // PB6 OFF
            delay_ms(5000);
            currentStep = 4;
        }

        if (currentStep == 4) {
            // Step 4: Turn off PB7, turn on PB6 for 3 seconds
            GPIO_PORTB_DATA_R &= ~(1 << 7); // PB7 OFF
            GPIO_PORTB_DATA_R |= (1 << 6);  // PB6 ON
            delay_ms(1500);
            currentStep = 5;
        }

        if (currentStep == 5) {
            // Step 5: Turn off PA4
            GPIO_PORTA_DATA_R &= ~(1 << 4); // PA4 OFF
            currentStep = 1; // Restart loop
        }
    }
}

// Delay function for milliseconds
// Modified delay_ms function to wait for switch release
void delay_ms(unsigned int time) {
    unsigned int i, j;
    for (i = 0; i < time; i++) {
        for (j = 0; j < 3180; j++) {
            // If the switch is pressed, skip the delay
            if ((GPIO_PORTF_DATA_R & (1 << 4)) == 0) {  // Switch pressed
                // Wait for the button to be released
                while ((GPIO_PORTF_DATA_R & (1 << 4)) == 0); // Wait for release
                currentStep = 1; // Restart the sequence from Step 1 after release
                return;  // Exit the delay function early
            }
						if ((GPIO_PORTF_DATA_R & (1 << 0)) == 0) {  // PF0 (SW1) pressed
                // Wait for the button (PF0) to be released
                while ((GPIO_PORTF_DATA_R & (1 << 0)) == 0); // Wait for release
                currentStep = 3; // Restart the sequence from Step 1 after release
                return;  // Exit the delay function early
            }
        }
    }
}



// Initialize Port F for onboard switches (SW1 and SW2)
void PortF_Init(void) {
    SYSCTL_RCGCGPIO_R |= (1 << 5);      // Enable clock for Port F
    while ((SYSCTL_PRGPIO_R & (1 << 5)) == 0); // Wait for Port F clock to stabilize

    GPIO_PORTF_LOCK_R = 0x4C4F434B;     // Unlock Port F for PF0
    GPIO_PORTF_CR_R |= (1 << 0) | (1 << 4); // Allow changes to PF0 and PF4

    GPIO_PORTF_DIR_R &= ~((1 << 0) | (1 << 4)); // Set PF0 and PF4 as inputs
    GPIO_PORTF_DEN_R |= (1 << 0) | (1 << 4);    // Enable digital functionality
    GPIO_PORTF_PUR_R |= (1 << 0) | (1 << 4);    // Enable pull-up resistors

    GPIO_PORTF_IS_R &= ~((1 << 0) | (1 << 4));  // Make PF0 and PF4 edge-sensitive
    GPIO_PORTF_IBE_R &= ~((1 << 0) | (1 << 4)); // Trigger on one edge
    GPIO_PORTF_IEV_R &= ~((1 << 0) | (1 << 4)); // Trigger on falling edge
    GPIO_PORTF_ICR_R = (1 << 0) | (1 << 4);     // Clear any prior interrupt
    GPIO_PORTF_IM_R |= (1 << 0) | (1 << 4);     // Unmask interrupts for PF0 and PF4

    NVIC_EN0_R |= (1 << 30);    // Enable interrupt for Port F in NVIC
    __enable_irq();             // Enable global interrupts
}

// Interrupt handler for Port F (SW1 and SW2)
void GPIOPortF_Handler(void) {
    if (GPIO_PORTF_RIS_R & (1 << 4)) {  // Check if SW1 (PF4) caused the interrupt
        GPIO_PORTF_ICR_R = (1 << 4);   // Clear interrupt flag for PF4
        currentStep = 1;               // Restart loop at Step 1
    }
    if (GPIO_PORTF_RIS_R & (1 << 0)) {  // Check if SW2 (PF0) caused the interrupt
        GPIO_PORTF_ICR_R = (1 << 0);   // Clear interrupt flag for PF0
        currentStep = 3;               // Jump to Step 3
    }
}
