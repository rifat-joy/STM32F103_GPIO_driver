
/* This source File Is a driver layer for STM32 GPIO and Interrupt 
 * Created : 13/05/22
 * Last Modified : 03/06/22
 * Consists :
			-> GPIO Configuration and Driver Functions
			-> Interrupt Configuration and Driver Functions
 */

#include "_HAL_GPIO.h"
#include <stdint.h>


uint32_t PINPOS[16] = { //takes to 1st bit in mode
	(0x00),		//PIN 0  -> 0
	(0x04),		//PIN 1  -> 4
	(0x08),		//PIN 2  -> 8
	(0x0C),		//PIN 3  -> 12
	(0x10),		//PIN 4  -> 16
	(0x14),		//PIN 5  -> 20
	(0x18),		//PIN 6  -> 24
	(0x1C),		//PIN 7  -> 28
	(0x00),		//PIN 8  -> 0
	(0x04),		//PIN 9  -> 4
	(0x08),		//PIN 10 -> 8
	(0x0C),		//PIN 11 -> 12
	(0x10),		//PIN 12 -> 16
	(0x14),		//PIN 13 -> 20
	(0x18),		//PIN 14 -> 24
	(0x1C),		//PIN 15 -> 28
};


static void config_pin (GPIO_TypeDef *port , uint32_t pinNumber , uint32_t mode_type)
{
	if(pinNumber >= 8)	// CONTROL HIGH REGISTER CHR
	{
		switch(mode_type)
		{
			//--------------OUTPUT & INPUT MODES CRH------------------
			
			case OUTPUT_GEN_PP | INPUT_ANALOG:
				port->CRH &= ~((1<<CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
			break;
			
			case OUTPUT_GEN_OD | INPUT_FLOATING:
				port->CRH &= ~(1<< CNF_POS_BIT2);
				port->CRH |= (1<< CNF_POS_BIT1);
			break;
			
			case OUTPUT_ALT_PP | INPUT_PU_PD:
				port->CRH |= OUTPUT_ALT_PP<<(CNF_POS_BIT1);
			break;
			
			case OUTPUT_ALT_OD:
				port->CRH |= OUTPUT_ALT_OD<<(CNF_POS_BIT1);
			break;
		}
	}
	else		//Control LOW Register
	{
		switch(mode_type)
		{
			//--------------OUTPUT & INPUT MODES CRL------------------
			
			case OUTPUT_GEN_PP | INPUT_ANALOG:
				port->CRL &= ~((1<<CNF_POS_BIT1) | (1<<CNF_POS_BIT2));
			break;
			
			case OUTPUT_GEN_OD | INPUT_FLOATING:
				port->CRL &= ~(1<< CNF_POS_BIT2);
				port->CRL |= (1<< CNF_POS_BIT1);
			break;
			
			case OUTPUT_ALT_PP | INPUT_PU_PD:
				port->CRL |= OUTPUT_ALT_PP<<(CNF_POS_BIT1);
			break;
			
			case OUTPUT_ALT_OD:
				port->CRL |= OUTPUT_ALT_OD<<(CNF_POS_BIT1);
			break;
		}
	}
}

static void config_pin_speed(GPIO_TypeDef *port, uint32_t pinNunber, uint32_t pinSpeed, uint32_t mode)
{
	if(pinNunber>8)
	{
		if(mode == INPUT_MODE)  //Setting CRH to input mode
			port->CRH &= ~(1<<(PINPOS[pinNunber]) | 1<<(PINPOS[pinNunber] + 1));
		else
			port->CRH |= (pinSpeed<< (PINPOS[pinNunber])); //Setting CRH at given speed
	} 
	else
	{
		if(mode == INPUT_MODE)
			port->CRL &= ~(1<<(PINPOS[pinNunber]) | 1<<(PINPOS[pinNunber] + 1));
		else
			port->CRL |= (pinSpeed<<(PINPOS[pinNunber])); //Setting CRL at given speed
	}
}

void gpio_write(GPIO_TypeDef *port, uint32_t pinNumber, uint8_t state)
{
	if (state)
	{
		port->BSRR = (1<<pinNumber);
	}
	else
	{
		port->BSRR = (1<<(pinNumber + 16));
	}
}

void gpio_toggle(GPIO_TypeDef *port, uint32_t pinNumber)
{
	port->ODR ^=(1<<pinNumber);
}

void gpio_init(GPIO_TYPE gpio_type)
{
	if(gpio_type.port == PORTA)
		GPIO_CLOCK_ENABLE_PORTA;
	
	if(gpio_type.port == PORTB)
		GPIO_CLOCK_ENABLE_PORTB;
	
	if(gpio_type.port == PORTC)
		GPIO_CLOCK_ENABLE_PORTC;
	
	if(gpio_type.port == PORTD)
		GPIO_CLOCK_ENABLE_PORTD;
	
	config_pin(gpio_type.port, gpio_type.pin, gpio_type.mode_type);
	config_pin_speed(gpio_type.port, gpio_type.pin, gpio_type.speed, gpio_type.mode);
}

/*______________Interrupt Functions______________*/

/* Configuring Alternate Functions for PORTS on AFIO_EXTICR */
void configure_gpio_interrupt(GPIO_TypeDef *port, uint32_t pinNumber, edge_select edge)
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enabling Alternate Function Clock
	
	/* Configuring for PORTA */
	if(port == PORTA)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PA;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PA;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PA;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PA;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PA;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PA;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PA;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PA;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PA;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PA;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PA;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PA;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PA;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PA;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PA;
			break;
		}
	}
	
	/* Configuring for PORTB */
	if(port == PORTB)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PB;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PB;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PB;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PB;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PB;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PB;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PB;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PB;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PB;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PB;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PB;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PB;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PB;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PB;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PB;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PB;
			break;
		}
	}
	
	/* Configuring for PORTC */
	if(port == PORTC)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PC;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PC;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PC;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PC;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PC;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PC;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PC;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PC;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PC;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PC;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PC;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PC;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PC;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PC;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PC;
			break;
		}
	}
	
	/* Configuring for PORTD */
	if(port == PORTD)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PD;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PD;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PD;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PD;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PD;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PD;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PD;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PD;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PD;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PD;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PD;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PD;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PD;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PD;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PD;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PD;
			break;
		}
	}
	
	/* Configuring for PORTE */
	if(port == PORTE)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PE;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PE;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PE;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PE;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PE;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PE;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PE;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PE;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PE;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PE;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PE;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PE;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PE;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PE;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PE;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PE;
			break;
		}
	}
	
	/* Configuring for PORTF */
	if(port == PORTF)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PF;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PF;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PF;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PF;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PF;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PF;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PF;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PF;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PF;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PF;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PF;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PF;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PF;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PF;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PF;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PF;
			break;
		}
	}
	
	/* Configuring for PORTG */
	if(port == PORTG)
	{
		switch(pinNumber)
		{
			/* Configuring External interrupt configuration register 1 */
			case 0:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PG;
			break;
			case 1:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI1_PG;
			break;
			case 2:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI2_PG;
			break;
			case 3:
				AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI3_PG;
			break;
			
			/* Configuring External interrupt configuration register 2 */
			case 4:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI4_PG;
			break;
			case 5:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI5_PG;
			break;
			case 6:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI6_PG;
			break;
			case 7:
				AFIO->EXTICR[1] = AFIO_EXTICR2_EXTI7_PG;
			break;
			
			/* Configuring External interrupt configuration register 3 */
			case 8:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI8_PG;
			break;
			case 9:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI9_PG;
			break;
			case 10:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI10_PG;
			break;
			case 11:
				AFIO->EXTICR[2] = AFIO_EXTICR3_EXTI11_PG;
			break;
			
			/* Configuring External interrupt configuration register 4 */
			case 12:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI12_PG;
			break;
			case 13:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PG;
			break;
			case 14:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI14_PG;
			break;
			case 15:
				AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI15_PG;
			break;
		}
	}
	
	/* Selecting Triggering Edge on EXTI_RTSR & EXTI_RTSR */
	if(edge == RISING_EDGE)
		EXTI->RTSR |= 1<<pinNumber;
	if(edge == FALLINF_EDGE)
		EXTI->FTSR |= 1<<pinNumber;
	if(edge == RISING_FALLING_EDGE)
	{
		EXTI->FTSR |= 1<<pinNumber;
		EXTI->RTSR |= 1<<pinNumber;
	}
	// Enable AFIO clock & SET PB1 as line for EXTI
}

/* Enabling GPIO interrupt on EXTI_IMR & NVIC*/
void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber)
{
	/* Enable interrupt on EXTI */
	EXTI->IMR |= 1<<pinNumber;
	/* Enable interrupt on NVIC */
	NVIC_EnableIRQ(irqNumber);
}

/* Clearing GPIO Interrupt Bit on EXIT_PR */
void clear_gpio_interrupt(uint32_t pinNumber)
{
	EXTI->PR |= 1<<pinNumber;
}




