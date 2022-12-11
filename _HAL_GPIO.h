
/* STM32 Peripheral driver Header [Rifat]
 * Created : 13/05/22
 * Last Modified : 03/06/22
 * Consists :
			-> GPIO Initialization
			-> GPIO Structs
			-> Interrupt enums
			-> Prototype functions
 */


#ifndef _HAL_GPIO
#define _HAL_GPIO

#include "stm32f10x.h"                  // Device header

#define LOW 	0
#define HIGH 	1

/********** GPIO Driver macros ***********/

//PORT names
#define PORTA				GPIOA
#define PORTB				GPIOB
#define PORTC				GPIOC
#define PORTD				GPIOD
#define PORTE				GPIOE
#define PORTF				GPIOF
#define PORTG				GPIOG

//PIN MODE
#define OUTPUT_MODE				((uint32_t) 0x01)
#define INPUT_MODE				((uint32_t) 0x02)

//INPUT MODES TYPE
#define INPUT_ANALOG			((uint32_t)0x00)
#define INPUT_FLOATING		((uint32_t)0x01)  //Default at reset
#define INPUT_PU_PD				((uint32_t)0x02)	//Input with pull up or pull down

//OUTPUT MODE TYPE
#define OUTPUT_GEN_PP			((uint32_t)0x00) //PP-> push-pull general purpouse
#define OUTPUT_GEN_OD			((uint32_t)0x01) //OD-> open-drain general purpouse
#define OUTPUT_ALT_PP			((uint32_t)0x02) //PP-> push-pull Alternate functions
#define OUTPUT_ALT_OD			((uint32_t)0x03) //open-drain Alternate functions

//OUTPUT SPEED TYPE
#define SPEED_2MHZ				((uint32_t)0x02)
#define SPEED_10MHZ				((uint32_t)0x01)
#define SPEED_50MHZ				((uint32_t)0x03)

//CLOCK ENABLING
#define GPIO_CLOCK_ENABLE_ALT_FUNC		(RCC->APB2ENR |= (1<<0))
#define GPIO_CLOCK_ENABLE_PORTA				(RCC->APB2ENR |= (1<<2))
#define GPIO_CLOCK_ENABLE_PORTB				(RCC->APB2ENR |= (1<<3))
#define GPIO_CLOCK_ENABLE_PORTC				(RCC->APB2ENR |= (1<<4))
#define GPIO_CLOCK_ENABLE_PORTD				(RCC->APB2ENR |= (1<<5))
//#define GPIO_CLOCK_ENABLE_PORTE		(RCC->APB2ENR |= (1<<6))

//HIGH BIT POSITION FOR CRH REGISTER CNFYG AND MODE
#define CNF_POS_BIT1		(PINPOS[pinNumber] + 2)
#define CNF_POS_BIT2		(PINPOS[pinNumber] + 3)


//Configuration Struction
typedef struct
{
	GPIO_TypeDef *port;
	uint32_t	pin;
	uint32_t	mode;
	uint32_t	mode_type;
	uint32_t	pull;
	uint32_t	speed;
	uint32_t	alt_func;
	
}GPIO_TYPE;

typedef enum
{
	RISING_EDGE,
	FALLINF_EDGE,
	RISING_FALLING_EDGE
}edge_select;

//Function prototypes
/****************** GPIO Configuration ****************************/
void gpio_init(GPIO_TYPE gpio_type);
static void config_pin (GPIO_TypeDef *port, uint32_t pinNumber, uint32_t mode_type);
static void config_pin_speed (GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinSpeed, uint32_t mode_type);

/**************** GPIO User pin function **************************/
void gpio_write(GPIO_TypeDef *port, uint32_t pinNumber, uint8_t state);
void gpio_toggle(GPIO_TypeDef *port, uint32_t pinNumber);

/***************** INTERRUPT FUNCTIONS ****************************/
void configure_gpio_interrupt(GPIO_TypeDef *port, uint32_t pinNumber, edge_select edge);
void enable_gpio_interrupt(uint32_t pinNumber, IRQn_Type irqNumber);
void clear_gpio_interrupt(uint32_t pinNumber);

#endif


