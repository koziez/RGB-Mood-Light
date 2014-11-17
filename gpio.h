/**************************************************************************//**
 * @file     gpio.h
 * @brief    Vanity header for the PAC5XXX GPIO
 * @version  V0.1
 * @date     1 April 2014
 *
 * @note
 * Copyright (C) 2014 Active Semiconductor. All rights reserved.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
#ifndef HYDRA_GPIO_H
#define HYDRA_GPIO_H

#include "pac5xxx.h"

// Type Enumerations

//		Pins

#define  PA0 	0
#define  PA1 	1
#define  PA2 	2
#define  PA3 	3
#define  PA4 	4
#define  PA5 	5
#define  PA6 	6
#define  PA7 	7
#define  PB0 	8
#define  PB1 	9
#define  PB2 	10
#define  PB3 	11
#define  PB4 	12
#define  PB5 	13
#define  PB6	14
#define  PB7 	15
#define  PC0 	16
#define  PC1 	17
#define  PC2 	18
#define  PC3 	19
#define  PC4 	20
#define  PC5 	21
#define  PC6 	22
#define  PC7 	23
#define  PD0 	24
#define  PD1 	25
#define  PD2 	26
#define  PD3 	27
#define  PD4 	28
#define  PD5 	29
#define  PD6 	30
#define  PD7 	31
#define  PE0 	32
#define  PE1 	33
#define  PE2 	34
#define  PE3 	35
#define  PE4 	36
#define  PE5 	37
#define  PE6 	38
#define  PE7 	39

//		Modes

#define INPUT 		0
#define OUTPUT		1
#define PULL_UP		2
#define PULL_DOWN 	3
#define RISING		4
#define	FALLING		5

//		States

#define HIGH	1
#define LOW		0




// Function Prototypes for support functions


/**
 * @brief	Configures the specified pin to behave either as an input or an output.
 *
 * @param	pin: the name of the pin whose mode you wish to set
 * 			mode: INPUT, OUTPUT, PULL_UP, or PULL_DOWN.
 * @return	none
 *
 */
void pinMode (int pin, int mode);


/**
 * @brief	Write a HIGH or a LOW value to a digital pin.
 * 			If the pin has been configured as an OUTPUT, its voltage will be set to the corresponding value: 5V for HIGH, 0V for LOW.
 * 			If the pin is configured as an INPUT, will enable (HIGH) or disable (LOW) the internal pull-up on the input pin.
 * 			It is recommended to set the pinMode() to INPUT_PULLUP to enable the internal pull-up resistor.
 * 			NOTE: If you do not set the pin to OUTPUT, and connect an LED to a pin, when calling digitalWrite(HIGH), the LED may appear dim.
 * 			Without explicitly setting pinMode(), digitalWrite() will have enabled the internal pull-up resistor, which acts like a large current-limiting resistor.
 *
 * @param	pin: the name of the pin
 * 			mode: HIGH or LOW
 * @return	none
 *
 */
void digitalWrite (int pin, int value);


/**
 * @brief	Reads the value from a specified digital pin, either HIGH or LOW.
 *
 * @param	pin: the number of the digital pin you want to read (int)
 * @return	HIGH or LOW
 *
 */
int digitalRead (int pin);


void delay (unsigned long ms);

void toggleWrite (int pin);

void clearPinEnable (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setPinEnable (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void clearPin (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setPin (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void clearPullup (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setPullup (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void clearPulldown (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setPulldown (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void togglePin (PAC5XXX_GPIO_TypeDef *gpio, int pin);

int checkOutputPin (PAC5XXX_GPIO_TypeDef *gpio, int pin);

int checkInputPin (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setPinMode (PAC5XXX_GPIO_TypeDef *gpio, int port, int mode);

int checkPinEnabled (PAC5XXX_GPIO_TypeDef *gpio, int pin);

int checkPin (PAC5XXX_GPIO_TypeDef *gpio, int pin);

//interrupts

typedef void (*isr)();

extern volatile isr GPIOA_ISR[8];
extern volatile isr GPIOB_ISR[8];
extern volatile isr GPIOC_ISR[8];
extern volatile isr GPIOD_ISR[8];
extern volatile isr GPIOE_ISR[8];

void attachInterrupt( int pin, isr ISR, int mode );

int checkFlag (PAC5XXX_GPIO_TypeDef *gpio, int pin);

int checkPolarity (PAC5XXX_GPIO_TypeDef *gpio, int pin);

int checkInterrupt (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setInterrupt (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void clearFlag (PAC5XXX_GPIO_TypeDef *gpio, int pin);

void setPolarity (PAC5XXX_GPIO_TypeDef *gpio, int pin, int mode);

void setInterruptISR( PAC5XXX_GPIO_TypeDef *gpio, volatile isr ISR_LIST[], isr ISR, int port, int mode);



void GPIO_HANDLER(PAC5XXX_GPIO_TypeDef *gpio, volatile isr list[]);

void GpioA_IRQHandler(void);

void GpioB_IRQHandler(void);

void GpioC_IRQHandler(void);

void GpioD_IRQHandler(void);

void GpioE_IRQHandler(void);



#endif // HYDRA_GPIO_H
