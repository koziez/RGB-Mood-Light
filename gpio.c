/**************************************************************************//**
 * @file     gpio.
 * @brief    Vanity source file for the PAC5XXX GPIO
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
#include "gpio.h"

void clearPinEnable (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->OUTEN.b &= ~(1 << pin);
}

void setPinEnable (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->OUTEN.b |= (1 << pin);
}

void clearPin (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->OUT.b &= ~(1 << pin);
}

void setPin (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->OUT.b |= (1 << pin);
}

void clearPullup (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->PU.b &= ~(1 << pin);
}

void setPullup (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->PU.b |= (1 << pin);
}

void clearPulldown (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->PD.b &= ~(1 << pin);
}

void setPulldown (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->PD.b |= (1 << pin);
}

void togglePin (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->OUT.b ^= (1 << pin);
}

int checkPinEnabled (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	uint32_t mask = (1 << pin);
	return ((gpio->OUTEN.b & mask) == mask);
}

int checkPin (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	uint32_t mask = (1 << pin);
	return ((gpio->IN.b & mask) == mask);
}

void setPinMode (PAC5XXX_GPIO_TypeDef *gpio, int port, int mode)
{
	switch(mode)
	{
	case OUTPUT:
		setPinEnable(gpio,port);
		break;
	case PULL_UP:
		if(gpio == PAC5XXX_GPIOC)
			PAC5XXX_GPIOC->INE.b |= (1 << port);
		else
			clearPinEnable(gpio,port);
		setPullup(gpio,port);
		break;
	case PULL_DOWN:
		if(gpio == PAC5XXX_GPIOC)
			PAC5XXX_GPIOC->INE.b |= (1 << port);
		else
			clearPinEnable(gpio,port);
		setPulldown(gpio,port);
		break;
	case INPUT:
		if(gpio == PAC5XXX_GPIOC)
			PAC5XXX_GPIOC->INE.b |= (1 << port);
		else
			clearPinEnable(gpio,port);
		break;
	default:
		break;
	}
}

void pinMode (int pin, int mode)
{
	int port;
	switch (pin)
	{
		case PA0:
			port = 0;
			PAC5XXX_GPIOA->PSEL.P0 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA1:
			port = 1;
			PAC5XXX_GPIOA->PSEL.P1 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA2:
			port = 2;
			PAC5XXX_GPIOA->PSEL.P2 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA3:
			port = 3;
			PAC5XXX_GPIOA->PSEL.P3 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA4:
			port = 4;
			PAC5XXX_GPIOA->PSEL.P4 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA5:
			port = 5;
			PAC5XXX_GPIOA->PSEL.P5 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA6:
			port = 6;
			PAC5XXX_GPIOA->PSEL.P6 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PA7:
			port = 7;
			PAC5XXX_GPIOA->PSEL.P7 = 0;
			setPinMode(PAC5XXX_GPIOA, port, mode);
			break;
		case PB0:
			port = 0;
			PAC5XXX_GPIOB->PSEL.P0 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB1:
			port = 1;
			PAC5XXX_GPIOB->PSEL.P1 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB2:
			port = 2;
			PAC5XXX_GPIOB->PSEL.P2 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB3:
			port = 3;
			PAC5XXX_GPIOB->PSEL.P3 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB4:
			port = 4;
			PAC5XXX_GPIOB->PSEL.P4 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB5:
			port = 5;
			PAC5XXX_GPIOB->PSEL.P5 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB6:
			port = 6;
			PAC5XXX_GPIOB->PSEL.P6 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PB7:
			port = 7;
			PAC5XXX_GPIOB->PSEL.P7 = 0;
			setPinMode(PAC5XXX_GPIOB, port, mode);
			break;
		case PC0:
			port = 0;
			PAC5XXX_GPIOC->PSEL.P0 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC1:
			port = 1;
			PAC5XXX_GPIOC->PSEL.P1 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC2:
			port = 2;
			PAC5XXX_GPIOC->PSEL.P2 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC3:
			port = 3;
			PAC5XXX_GPIOC->PSEL.P3 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC4:
			port = 4;
			PAC5XXX_GPIOC->PSEL.P4 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC5:
			port = 5;
			PAC5XXX_GPIOC->PSEL.P5 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC6:
			port = 6;
			PAC5XXX_GPIOC->PSEL.P6 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PC7:
			port = 7;
			PAC5XXX_GPIOC->PSEL.P7 = 0;
			setPinMode(PAC5XXX_GPIOC, port, mode);
			break;
		case PD0:
			port = 0;
			PAC5XXX_GPIOD->PSEL.P0 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD1:
			port = 1;
			PAC5XXX_GPIOD->PSEL.P1 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD2:
			port = 2;
			PAC5XXX_GPIOD->PSEL.P2 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD3:
			port = 3;
			PAC5XXX_GPIOD->PSEL.P3 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD4:
			port = 4;
			PAC5XXX_GPIOD->PSEL.P4 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD5:
			port = 5;
			PAC5XXX_GPIOD->PSEL.P5 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD6:
			port = 6;
			PAC5XXX_GPIOD->PSEL.P6 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PD7:
			port = 7;
			PAC5XXX_GPIOD->PSEL.P7 = 0;
			setPinMode(PAC5XXX_GPIOD, port, mode);
			break;
		case PE0:
			port = 0;
			PAC5XXX_GPIOE->PSEL.P0 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE1:
			port = 1;
			PAC5XXX_GPIOE->PSEL.P1 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE2:
			port = 2;
			PAC5XXX_GPIOE->PSEL.P2 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE3:
			port = 3;
			PAC5XXX_GPIOE->PSEL.P3 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE4:
			port = 4;
			PAC5XXX_GPIOE->PSEL.P4 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE5:
			port = 5;
			PAC5XXX_GPIOE->PSEL.P5 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE6:
			port = 6;
			PAC5XXX_GPIOE->PSEL.P6 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		case PE7:
			port = 7;
			PAC5XXX_GPIOE->PSEL.P7 = 0;
			setPinMode(PAC5XXX_GPIOE, port, mode);
			break;
		default:
			break;
	}
}

void setOutputWrite (PAC5XXX_GPIO_TypeDef *gpio, int port, int value)
{
	switch(value)
	{
	case HIGH:
		setPin(gpio, port);				//write HIGH
		break;
	case LOW:
		clearPin(gpio, port);			//write LOW
		break;
	default:
		break;
	}
}

void setInputWrite (PAC5XXX_GPIO_TypeDef *gpio, int port, int value)
{
	switch(value)
	{
	case HIGH:
		setPullup(gpio, port);				//enable pull-up
		break;
	case LOW:
		clearPullup(gpio, port);			//disable pull-up
		break;
	default:
		break;
	}
}

void digitalWrite (int pin, int value)
{
	int port;
	switch (pin)
	{
		case PA0:
			port = 0;
			PAC5XXX_GPIOA->PSEL.P0 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA1:
			port = 1;
			PAC5XXX_GPIOA->PSEL.P1 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA2:
			port = 2;
			PAC5XXX_GPIOA->PSEL.P2 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA3:
			port = 3;
			PAC5XXX_GPIOA->PSEL.P3 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA4:
			port = 4;
			PAC5XXX_GPIOA->PSEL.P4 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA5:
			port = 5;
			PAC5XXX_GPIOA->PSEL.P5 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA6:
			port = 6;
			PAC5XXX_GPIOA->PSEL.P6 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PA7:
			port = 7;
			PAC5XXX_GPIOA->PSEL.P7 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOA, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOA, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOA, port, value);
			}
			break;
		case PB0:
			port = 0;
			PAC5XXX_GPIOB->PSEL.P0 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB1:
			port = 1;
			PAC5XXX_GPIOB->PSEL.P1 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB2:
			port = 2;
			PAC5XXX_GPIOB->PSEL.P2 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB3:
			port = 3;
			PAC5XXX_GPIOB->PSEL.P3 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB4:
			port = 4;
			PAC5XXX_GPIOB->PSEL.P4 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB5:
			port = 5;
			PAC5XXX_GPIOB->PSEL.P5 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB6:
			port = 6;
			PAC5XXX_GPIOB->PSEL.P6 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PB7:
			port = 7;
			PAC5XXX_GPIOB->PSEL.P7 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOB, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOB, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOB, port, value);
			}
			break;
		case PC0:
			port = 0;
			PAC5XXX_GPIOC->PSEL.P0 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC1:
			port = 1;
			PAC5XXX_GPIOC->PSEL.P1 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC2:
			port = 2;
			PAC5XXX_GPIOC->PSEL.P2 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC3:
			port = 3;
			PAC5XXX_GPIOC->PSEL.P3 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC4:
			port = 4;
			PAC5XXX_GPIOC->PSEL.P4 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC5:
			port = 5;
			PAC5XXX_GPIOC->PSEL.P5 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC6:
			port = 6;
			PAC5XXX_GPIOC->PSEL.P6 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PC7:
			port = 7;
			PAC5XXX_GPIOC->PSEL.P7 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOC, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOC, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOC, port, value);
			}
			break;
		case PD0:
			port = 0;
			PAC5XXX_GPIOD->PSEL.P0 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD1:
			port = 1;
			PAC5XXX_GPIOD->PSEL.P1 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD2:
			port = 2;
			PAC5XXX_GPIOD->PSEL.P2 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD3:
			port = 3;
			PAC5XXX_GPIOD->PSEL.P3 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD4:
			port = 4;
			PAC5XXX_GPIOD->PSEL.P4 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD5:
			port = 5;
			PAC5XXX_GPIOD->PSEL.P5 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD6:
			port = 6;
			PAC5XXX_GPIOD->PSEL.P6 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PD7:
			port = 7;
			PAC5XXX_GPIOD->PSEL.P7 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOD, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOD, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOD, port, value);
			}
			break;
		case PE0:
			port = 0;
			PAC5XXX_GPIOE->PSEL.P0 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE1:
			port = 1;
			PAC5XXX_GPIOE->PSEL.P1 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE2:
			port = 2;
			PAC5XXX_GPIOE->PSEL.P2 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE3:
			port = 3;
			PAC5XXX_GPIOE->PSEL.P3 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE4:
			port = 4;
			PAC5XXX_GPIOE->PSEL.P4 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE5:
			port = 5;
			PAC5XXX_GPIOE->PSEL.P5 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE6:
			port = 6;
			PAC5XXX_GPIOE->PSEL.P6 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		case PE7:
			port = 7;
			PAC5XXX_GPIOE->PSEL.P7 = 0;
			if(checkPinEnabled(PAC5XXX_GPIOE, port))		//output
			{
				setOutputWrite(PAC5XXX_GPIOE, port, value);
			}
			else											//input
			{
				setInputWrite(PAC5XXX_GPIOE, port, value);
			}
			break;
		default:
			break;
	}
}

void toggleWrite (int pin)
{
	int port;
	switch (pin)
	{
		case PA0:
			port = 0;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA1:
			port = 1;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA2:
			port = 2;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA3:
			port = 3;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA4:
			port = 4;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA5:
			port = 5;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA6:
			port = 6;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PA7:
			port = 7;
			togglePin(PAC5XXX_GPIOA, port);
			break;
		case PB0:
			port = 0;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB1:
			port = 1;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB2:
			port = 2;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB3:
			port = 3;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB4:
			port = 4;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB5:
			port = 5;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB6:
			port = 6;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PB7:
			port = 7;
			togglePin(PAC5XXX_GPIOB, port);
			break;
		case PC0:
			port = 0;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC1:
			port = 1;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC2:
			port = 2;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC3:
			port = 3;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC4:
			port = 4;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC5:
			port = 5;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC6:
			port = 6;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PC7:
			port = 7;
			togglePin(PAC5XXX_GPIOC, port);
			break;
		case PD0:
			port = 0;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD1:
			port = 1;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD2:
			port = 2;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD3:
			port = 3;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD4:
			port = 4;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD5:
			port = 5;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD6:
			port = 6;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PD7:
			port = 7;
			togglePin(PAC5XXX_GPIOD, port);
			break;
		case PE0:
			port = 0;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE1:
			port = 1;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE2:
			port = 2;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE3:
			port = 3;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE4:
			port = 4;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE5:
			port = 5;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE6:
			port = 6;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		case PE7:
			port = 7;
			togglePin(PAC5XXX_GPIOE, port);
			break;
		default:
			break;
	}
}

int digitalRead (int pin)
{
	int port;
	switch (pin)
	{
		case PA0:
			port = 0;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA1:
			port = 1;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA2:
			port = 2;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA3:
			port = 3;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA4:
			port = 4;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA5:
			port = 5;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA6:
			port = 6;
			return checkPin(PAC5XXX_GPIOA, port);
		case PA7:
			port = 7;
			return checkPin(PAC5XXX_GPIOA, port);
		case PB0:
			port = 0;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB1:
			port = 1;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB2:
			port = 2;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB3:
			port = 3;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB4:
			port = 4;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB5:
			port = 5;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB6:
			port = 6;
			return checkPin(PAC5XXX_GPIOB, port);
		case PB7:
			port = 7;
			return checkPin(PAC5XXX_GPIOB, port);
		case PC0:
			port = 0;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC1:
			port = 1;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC2:
			port = 2;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC3:
			port = 3;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC4:
			port = 4;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC5:
			port = 5;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC6:
			port = 6;
			return checkPin(PAC5XXX_GPIOC, port);
		case PC7:
			port = 7;
			return checkPin(PAC5XXX_GPIOC, port);
		case PD0:
			port = 0;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD1:
			port = 1;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD2:
			port = 2;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD3:
			port = 3;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD4:
			port = 4;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD5:
			port = 5;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD6:
			port = 6;
			return checkPin(PAC5XXX_GPIOD, port);
		case PD7:
			port = 7;
			return checkPin(PAC5XXX_GPIOD, port);
		case PE0:
			port = 0;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE1:
			port = 1;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE2:
			port = 2;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE3:
			port = 3;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE4:
			port = 4;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE5:
			port = 5;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE6:
			port = 6;
			return checkPin(PAC5XXX_GPIOE, port);
		case PE7:
			port = 7;
			return checkPin(PAC5XXX_GPIOE, port);
		default:
			break;
	}
	return 0;
}

void delay (unsigned long ms)
{
	unsigned long cycles = 133.33 * ms;
	unsigned long time;
	for (time = 0; time < cycles; time++);
}

//interrupts

int checkFlag (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	uint32_t mask = (1 << pin);
	return ((gpio->INTF.b & mask) == mask);
}

int checkPolarity (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	uint32_t mask = (1 << pin);
	return ((gpio->INTP.b & mask) == mask);
}

int checkInterrupt (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	uint32_t mask = (1 << pin);
	return ((gpio->INTE.b & mask) == mask);
}

void setInterrupt (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->INTE.b |= (1 << pin);
}

void clearFlag (PAC5XXX_GPIO_TypeDef *gpio, int pin){
	gpio->INTF.b |= (1 << pin);
}

void setPolarity (PAC5XXX_GPIO_TypeDef *gpio, int pin, int mode){
	switch(mode)
	{
	case RISING:
		gpio->INTP.b |= (1 << pin);
		break;
	case FALLING:
		gpio->INTP.b &= ~(1 << pin);
		break;
	default:
		break;
	}
}

void setInterruptISR( PAC5XXX_GPIO_TypeDef *gpio, volatile isr ISR_LIST[], isr ISR, int port, int mode)
{
	setInterrupt(gpio, port);						//enable
	setPolarity(gpio, port, mode);					//polarity
	ISR_LIST[port] = ISR;							//set isr
	clearFlag(gpio, port);							//clear flags - write 1 to clear
	gpio->INTM.b = 0;								//clear masks
}


void attachInterrupt( int pin, isr ISR, int mode )
{
	__disable_irq();
	int port;
	switch(pin)
	{
	case PA0:
		port = 0;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA1:
		port = 1;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA2:
		port = 2;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA3:
		port = 3;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA4:
		port = 4;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA5:
		port = 5;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA6:
		port = 6;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PA7:
		port = 7;
		setInterruptISR(PAC5XXX_GPIOA, GPIOA_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioA_IRQn);
		break;
	case PB0:
		port = 0;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB1:
		port = 1;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB2:
		port = 2;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB3:
		port = 3;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB4:
		port = 4;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB5:
		port = 5;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB6:
		port = 6;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PB7:
		port = 7;
		setInterruptISR(PAC5XXX_GPIOB, GPIOB_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioB_IRQn);
		break;
	case PC0:
		port = 0;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC1:
		port = 1;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC2:
		port = 2;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC3:
		port = 3;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC4:
		port = 4;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC5:
		port = 5;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC6:
		port = 6;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PC7:
		port = 7;
		setInterruptISR(PAC5XXX_GPIOC, GPIOC_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioC_IRQn);
		break;
	case PD0:
		port = 0;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD1:
		port = 1;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD2:
		port = 2;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD3:
		port = 3;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD4:
		port = 4;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD5:
		port = 5;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD6:
		port = 6;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PD7:
		port = 7;
		setInterruptISR(PAC5XXX_GPIOD, GPIOD_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioD_IRQn);
		break;
	case PE0:
		port = 0;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE1:
		port = 1;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE2:
		port = 2;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE3:
		port = 3;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE4:
		port = 4;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE5:
		port = 5;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE6:
		port = 6;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	case PE7:
		port = 7;
		setInterruptISR(PAC5XXX_GPIOE, GPIOE_ISR, ISR, port, mode);
		NVIC_EnableIRQ(GpioE_IRQn);
		break;
	default:
		break;
	}

	__enable_irq();
}

volatile isr GPIOA_ISR[8] = {0};
volatile isr GPIOB_ISR[8] = {0};
volatile isr GPIOC_ISR[8] = {0};
volatile isr GPIOD_ISR[8] = {0};
volatile isr GPIOE_ISR[8] = {0};

void GPIO_HANDLER(PAC5XXX_GPIO_TypeDef *gpio, volatile isr list[])
{
	int port;
	for(port = 0; port < 8; port++)
	{
		if(checkInterrupt(gpio,port) && checkFlag(gpio,port))			//check enabled and flag
		{
			if(list[port])
				list[port]();													//run ISR if set
			clearFlag(gpio, port);												//clear flag
		}
	}
	gpio->INTF.b |= 0xFF;								//clear all flags
}

void GpioA_IRQHandler(void)
{
   GPIO_HANDLER(PAC5XXX_GPIOA,GPIOA_ISR);
}

void GpioB_IRQHandler(void)
{
   GPIO_HANDLER(PAC5XXX_GPIOB,GPIOB_ISR);
}

void GpioC_IRQHandler(void)
{
   GPIO_HANDLER(PAC5XXX_GPIOC,GPIOC_ISR);
}

void GpioD_IRQHandler(void)
{
   GPIO_HANDLER(PAC5XXX_GPIOD,GPIOD_ISR);
}

void GpioE_IRQHandler(void)
{
   GPIO_HANDLER(PAC5XXX_GPIOE,GPIOE_ISR);
}

