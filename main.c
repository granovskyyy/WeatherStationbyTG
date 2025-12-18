#include "MKL05Z4.h" 
#define LED_RED    0 
static const uint32_t MaskLED[] = {1UL << 8}; 
static void LED_Init (void) { 
SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; 
PORTB->PCR[8] = PORT_PCR_MUX(1UL); 
PTB->PDDR |= MaskLED[LED_RED]; 
} 
	static void LED_Blink (void) { 
	uint32_t i = 0; 
	PTB->PTOR = MaskLED[LED_RED]; 
	for(i = 0; i < 3000000; i++)__NOP(); 
	} 
int main(void){ 
	LED_Init (); 
	while(1) { 
		LED_Blink (); 
	} 
} 
