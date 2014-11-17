/******************************************************************************
 * @file     main.c
 * @brief
 *
 * @note
 * Copyright (C) 2014, Active-Semi International
 *
 * THIS SOFTWARE IS SUBJECT TO A SOURCE CODE LICENSE AGREEMENT WHICH PROVIDES,
 * AMONG OTHER THINGS:  (i) THAT IT CAN BE USED ONLY TO ADAPT THE LICENSEE'S
 * APPLICATION TO PAC PROCESSORS SUPPLIED BY ACTIVE-SEMI INTERNATIONAL;
 * (ii) THAT  IT IS PROVIDED "AS IS" WITHOUT WARRANTY;  (iii) THAT
 * ACTIVE-SEMICONDUCTOR IS NOT LIABLE FOR ANY INDIRECT DAMAGES OR FOR DIRECT
 * DAMAGES EXCEEDING US$1,500;  AND (iv) THAT IT CAN BE DISCLOSED TO AND USED
 * ONLY BY CERTAIN AUTHORIZED PERSONS.
 *
 ******************************************************************************/

#include "pac5xxx_driver_uart.h"
#include "pac5xxx_driver_gpio.h"
#include "pac5xxx_driver_timer.h"
#include "gpio.h"
typedef enum { false, true } bool;

// UART_RX[0] = 0x71 						//First Byte
// UART_RX[1] = Opcode						//Command Number from 0 to 255
// UART_RX[2] = BYTE0						//Byte 0 bits[07:00]
// UART_RX[3] = Checksum					//8 bit Sum of previous bytes
#define UART_MESSAGE_SIZE	4
#define START_BYTE_REQUEST 0x71        //q
#define	START_BYTE_RESPONSE 0x70	   //p

//CONSTANTS
#define MAX 255
#define MIN 0
#define RED 0
#define GREEN 1
#define BLUE 2

//VARIABLES
int LED_RED = PA0;
int LED_GREEN = PA2;
int LED_BLUE = PA4;
int PERIOD = 512;	//120 Hz
int CYCLE = 100000;	//~85 Hz
int w[3] = {0,1,2};

//WORKING VARIABLES
int v[3] = {255,255,255};
int d[3] = {1,1,1};
int c[3] = {0,0,0};
bool LEDOn = false;
bool MoodOn = false;
bool initialized = false;

uint8_t uart_rx_buffer[UART_MESSAGE_SIZE];
uint8_t uart_tx_buffer[UART_MESSAGE_SIZE];

static int uart_rx_byte_count = 0;
static int uart_tx_byte_count = 0;

void uart_message_process(void);
static uint8_t uart_checksum(uint8_t* pBuffer);
void init(void);
void update(void);


/**
 * @brief  On this code example the main() function is in charge of configuring the Clock System,
 * configuring the GPIO, configuring the UART and configuring interrupts. Serial communications will
 * be handled by the interrupt service routine (ISR).
 *
 * @return none
 *
 */

int main(void)
{
	__disable_irq();
	init();
	__enable_irq();

	update();

	while(1);
}

void init(void)
{


	// Configure PORTA IO
	pinMode(LED_RED,OUTPUT);
	pinMode(LED_GREEN,OUTPUT);
	pinMode(LED_BLUE,OUTPUT);


	// Configure UART IO
	pac5xxx_uart_io_config();
	pac5xxx_gpio_out_pull_up_e(0x0C);

	// Configure UART peripheral
	pac5xxx_uart_config_LCR(UART_BPC_8,						// Bits per character
							UART_STOP_BITS_1,				// Stop Bits
							0,								// Parity Enabled
							0,								// Parity Type
							0,								// Stick Parity
							UART_BRKCTL_NORMAL,				// Break Control
							1);								// Divisor Latch Access

	// Configure UART clock for baud rate
	//We will use the 1% accurate 4 MHz Reference Clock as our Fast Clock
	pac5xxx_sys_ccs_config(CCSCTL_CLKIN_CLKREF, CCSCTL_ACLKDIV_DIV1, CCSCTL_HCLKDIV_DIV1);

	// BAUD = 4 MHz/(9600*16) = 26.04167 -> divisor = 26; fractional: 0.04167*256 = 10.67
	pac5xxx_uart_config_divisor_latch(26);
	pac5xxx_uart_config_fractional_divisor(10);
	
	// Configure UART peripheral for access to FIFO from this point on
	pac5xxx_uart_fifo_access();

	// Enable FIFOs, so that the interrupts will work properly
	pac5xxx_uart_fifo_enable(1);

	// Enable receive data available interrupt and NVIC for UART interrupts
	pac5xxx_uart_int_enable_RDAI(1);
	NVIC_SetPriority(UART_IRQn, 3);
	NVIC_EnableIRQ(UART_IRQn);

	SysTick->LOAD = CYCLE;
	SysTick->VAL = 0;
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk +  SysTick_CTRL_CLKSOURCE_Msk + SysTick_CTRL_TICKINT_Msk);
}


void UART_IRQHandler(void)
{
	uint8_t int_type = PAC5XXX_UART->IIR.IID;
	uint8_t data = pac5xxx_uart_read();

	if (int_type == UARTIIR_IID_TX_HOLD_EMPTY)
	{
		pac5xxx_uart_write(uart_tx_buffer[uart_tx_byte_count++]);	// Send next byte from uart_tx_buffer

		if (uart_tx_byte_count == UART_MESSAGE_SIZE)		// Last byte of message to transmit
		{
			pac5xxx_uart_int_enable_THREI(0);				// Disable transmit interrupt
			pac5xxx_uart_int_enable_RDAI(1);				// Enable receive data interrupt for next incoming message
			uart_tx_byte_count = 0;
		}
	}
	else
	{
		if (uart_rx_byte_count == 0)						// For the first byte received, the start byte must be 0x89
		{
			if (data != START_BYTE_REQUEST)					// Input message must always start with 0xA5
				return;
		}

		if(!initialized)
			initialized = true;

		uart_rx_buffer[uart_rx_byte_count++] = data;		// Store data in buffer and increment index

		if (uart_rx_byte_count == UART_MESSAGE_SIZE)		// Received all bytes of message
		{
			uart_message_process();							// Process incoming message in uart_rx_buffer and put response in uart_tx_buffer
			uart_rx_byte_count = 0;
			uart_tx_byte_count = 0;

			pac5xxx_uart_int_enable_RDAI(0);				// Disable receive data interrupt
			pac5xxx_uart_rx_fifo_reset();					// Reset RX FIFO, to clear RDAI interrupt

			pac5xxx_uart_int_enable_THREI(1);				// Enable transmit interrupt to send response to host
		}
	}
}

void SysTick_Handler(void)
{
	if(MoodOn)
	{
		changeColors();
	}
	update();
}

void update(void)
{
	if(LEDOn)
	{
		initLights();

	}else{
		digitalWrite(LED_RED,LOW);
		digitalWrite(LED_GREEN,LOW);
		digitalWrite(LED_BLUE,LOW);
	}

}

void uart_message_process(void)
{
	uint32_t rx_data;

	// Full message received in uart_rx_buffer, process and then queue up transmit (response) message
	uart_tx_buffer[0] = START_BYTE_RESPONSE;				// Response message type is always 0x5A

	// Put data from uart_rx_buffer into word for processing
	//rx_data = uart_rx_buffer[2];

	// Validate checksum. Checksum on last byte equals the sum of all previous bytes.
	// Transmit buffer byte 1 is used to specify if a checksum error was received.
	// If checksum is correct, serial command is executed.
	if (uart_checksum((uint8_t*)&uart_rx_buffer) != uart_rx_buffer[UART_MESSAGE_SIZE - 1])
	{
		uart_tx_buffer[1] = uart_rx_buffer[1] + 0x88;		// Response message code is received message code + 0x88
	}
	else
	{
		//serial command code to execute
		if(uart_rx_buffer[1] == 0xAA)
		{
			if(uart_rx_buffer[2] == 0x01)
			{
				LEDOn = true;
				update();
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
			}
			else
			{
				if(uart_rx_buffer[2] == 0x00)
				{
					LEDOn = false;
					update();
					uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
				}
			}
		}
		switch(uart_rx_buffer[1])
		{
		case 0xAA:
			if(uart_rx_buffer[2] == 0x01)
			{
				LEDOn = true;
				update();
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
			}
			else
			{
				if(uart_rx_buffer[2] == 0x00)
				{
					LEDOn = false;
					update();
					uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
				}
			}
			break;
		case 0xBB:
			if(uart_rx_buffer[2] == 0x01)
			{
				MoodOn = true;
				update();
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
			}
			else
			{
				if(uart_rx_buffer[2] == 0x00)
				{
					MoodOn = false;
					update();
					uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
				}
			}
			break;
		case 0xC1:
			if(!MoodOn)
			{
				v[0] = (int)uart_rx_buffer[2];
				update();
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
			}
			else
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x87;
			break;
		case 0xC2:
			if(!MoodOn)
			{
				v[1] = (int)uart_rx_buffer[2];
				update();
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
			}
			else
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x87;
			break;
		case 0xC3:
			if(!MoodOn)
			{
				v[2] = (int)uart_rx_buffer[2];
				update();
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x80;
			}
			else
				uart_tx_buffer[1] = uart_rx_buffer[1] + 0x87;
			break;
		default:
			uart_tx_buffer[1] = uart_rx_buffer[1] + 0x89;
		}
	}

	// Calculate checksum for outgoing message
	uart_tx_buffer[UART_MESSAGE_SIZE - 1] = uart_checksum((uint8_t*)&uart_tx_buffer);
}

static uint8_t uart_checksum(uint8_t* pBuffer)
{
	uint8_t checksum = 0;
	int i;

	for (i = 0; i < (UART_MESSAGE_SIZE - 1); i++)
		checksum += *pBuffer++;

	return checksum;
}

void changeColors()
{
	int i;
	for(i = 0; i < 3; i++)
	{
		//implement countdown
		if(c[i] == 0 && (v[i] == MIN || v[i] == MAX))
		{
			c[i] = MAX * w[i];
			d[i] *= -1;
			v[i] += d[i];
		}
		if(c[i] > 0)
		{
			c[i]--;
		}
		else
		{
			v[i] += d[i];
		}
	}
}

void initLights()
{
	initRed();
	initGreen();
	initBlue();
}

void initRed()
{
	pac5xxx_timer_io_select_pwma0_pa0();
	pac5xxx_timer_clock_config(TimerA, TxCTL_CS_ACLK, TxCTL_PS_DIV16);
	pac5xxx_timer_base_config(TimerA, PERIOD, 0, TxCTL_MODE_UP, 0);
	float value = (float)v[RED] / MAX;
	pac5xxx_timer_a_ccctr0_value_set(value * PERIOD);
}

void initGreen()
{
	pac5xxx_timer_io_select_pwma2_pa2();
	pac5xxx_timer_clock_config(TimerA, TxCTL_CS_ACLK, TxCTL_PS_DIV16);
	pac5xxx_timer_base_config(TimerA, PERIOD, 0, TxCTL_MODE_UP, 0);
	float value = (float)v[GREEN] / MAX;
	pac5xxx_timer_a_ccctr2_value_set(value * PERIOD);
}

void initBlue()
{
	pac5xxx_timer_io_select_pwma5_pa4();
	pac5xxx_timer_clock_config(TimerA, TxCTL_CS_ACLK, TxCTL_PS_DIV16);
	pac5xxx_timer_base_config(TimerA, PERIOD, 0, TxCTL_MODE_UP, 0);
	float value = (float)v[BLUE] / MAX;
	pac5xxx_timer_a_ccctr5_value_set(value * PERIOD);
}

void checkConnected()
{
	uart_tx_buffer[0] = 0x41;
	uart_tx_buffer[1] = 0x54;
	uart_tx_buffer[2] = 0x00;
	uart_tx_buffer[3] = 0x00;
	uart_tx_byte_count = 0;
	pac5xxx_uart_int_enable_THREI(1);
}
