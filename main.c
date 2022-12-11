
/* This File is a test of "_HAL_GPIO.h" 
 * STM32 Peripheral driver GPIO and Interrupt [Rifat]
 */

#include "stm32f10x.h"                  // Device header
#include "_HAL_GPIO.h"

int main()
{
	//Configuring GPIO for General purpouse (blink)
	GPIO_TYPE myGPIO;
	myGPIO.port = PORTC;
	myGPIO.pin = 13;
	myGPIO.mode = OUTPUT_MODE;
	myGPIO.mode_type = OUTPUT_GEN_PP;
	myGPIO.speed = SPEED_50MHZ;
	
	gpio_init(myGPIO);
	
	//Configuring GPIO for Interrupt
	configure_gpio_interrupt(PORTB,4,RISING_EDGE);
	enable_gpio_interrupt(4,EXTI4_IRQn);
	
	
	while(1)
	{
		//gpio_toggle(PORTC,13);
		gpio_write(PORTC,13,LOW);
		for(int i=0;i<5000000;i++);   // Emulating delay Counting numbers
		gpio_write(PORTC,13,HIGH);		
		for(int i=0;i<5000000;i++);		// Emulating delay Counting numbers
		
	}
	
}

/* Interrupt Handler */
void EXTI4_IRQ_IRQHandler()
{
	clear_gpio_interrupt(4); //clearing pending bit
	
}


