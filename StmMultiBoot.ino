/* 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "StmMultiBoot.h"
#include "stk500.h"

#define _FLASH_PROG		1

// Temp
#define SIGNATURE_0		0x1E
#define SIGNATURE_1		0x55 //0x97
#define SIGNATURE_2		0xAA //0x02
#define SIGNATURE_3		0x97
#define SIGNATURE_4		0x02

#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 7

// Macro to toggle the LED
#define __MULTI_TOGGLE_LED() HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1)

// Structure for configuring GPIO pins
GPIO_InitTypeDef GPIO_InitStruct;

// Structure for erasing flash pages
FLASH_EraseInitTypeDef EraseInitStruct;

// Boundaries of program flash space
#ifdef STM32F103xB
	#define PROGFLASH_START 0x08002000
	#define PROGFLASH_END 0x08020000
#endif
#ifdef STM32F303xC
	#define PROGFLASH_START 0x08002000
	#define PROGFLASH_END 0x08040000
#endif

uint32_t ResetReason ;
uint32_t LongCount ;
uint8_t Buff[512] ;

uint8_t NotSynced ;
uint8_t SyncCount ;
uint8_t Port ;

// Handle for TIM2
static TIM_HandleTypeDef Timer2Handle = {
	.Instance = TIM2
};

static void Timer_Init()
{	
	__HAL_RCC_TIM2_CLK_ENABLE();
	Timer2Handle.Instance = TIM2;

	// 72000000 / ((479+1)*(9999+1)) = 15Hz
	Timer2Handle.Init.Prescaler = 479;
	Timer2Handle.Init.Period = 9999;
	Timer2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Timer2Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	Timer2Handle.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&Timer2Handle);
	HAL_TIM_Base_Start(&Timer2Handle);
}

static void GPIO_Init()
{
	// Enable the clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	#ifdef __HAL_RCC_AFIO_CLK_ENABLE
		__HAL_RCC_AFIO_CLK_ENABLE();		// The AFIO clock only exists on the F103
	#endif

	// Set PA0, 4-7 (HIGH)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

	// Configure pins PA0, 4-7 as inputs - PA0 is BIND button, PA4-7 are the rotary dial
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PA1 as output (LED)
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PB1 and PB3 as output (enable/disable serial inverter)
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Disable JTAG
	//__HAL_AFIO_REMAP_SWJ_DISABLE();		// Disable JTAG and SWD - use for released version
	__HAL_AFIO_REMAP_SWJ_NOJTAG();			// Disable JTAG but keep SWD enabled - use for development/debugging
}

static void Serial_Init()
{
	// Enable the clocks
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();

	// USART2 - TX=PA2, RX=PA9 - only RX is used 
	// Configure PA9 as alternate function USART2_RX
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

	#ifdef STM32F303xC
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	#endif

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// USART3 - TX=PB10, RX=PB11 - only TX is used
	// Configure PB10 as alternate function USART3_TX
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

	#ifdef STM32F303xC
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	#endif

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	USART1->BRR = 72000000 / 57600;
	USART1->CR1 = 0x200C;
	USART2->BRR = 36000000 / 57600;
	USART2->CR1 = 0x200C;
	USART2->CR2 = 0;
	USART2->CR3 = 0;
	USART3->BRR = 36000000 / 57600;
	USART3->CR1 = 0x200C;
	USART3->CR2 = 0;
	USART3->CR3 = 0;

	// Start with the serial inverter disabled
	DisableSerialInverter();
}

/* Disables the hardware serial port inverter */
static void DisableSerialInverter()
{
	// Set PB1 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// Clear PB3 (LOW)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

}

/* Enables the hardware serial port inverter */
static void EnableSerialInverter()
{
	// Set PB1 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// Set PB3 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

static uint32_t CheckForBindButton()
{
	uint8_t ch;

	// Would this work instead?
	// ch = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
	ch = GPIOA->IDR & 0xF1;
	return (ch != 0xF0) ? 0 : 1;
}

/* Returns 1 if the device was reset via a software request, 0 for any other reset reason */
static uint32_t SoftwareResetReason()
{
	// Get the reset reason
	ResetReason = RCC->CSR;

	// Clear the reset flag
	RCC->CSR |= RCC_CSR_RMVF;


	return (ResetReason & RCC_CSR_SFTRSTF) ? 1 : 0;
}

void disableInterrupts()
{
	__disable_irq() ;
	HAL_NVIC_DisableIRQ(USART1_IRQn) ;
	HAL_NVIC_DisableIRQ(USART2_IRQn) ;
	HAL_NVIC_DisableIRQ(USART3_IRQn) ;

	// Disable STM32F103-only interrupts
	#ifdef STM32F103xB
		HAL_NVIC_DisableIRQ(TIM1_BRK_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM1_CC_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM1_UP_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM1_TRG_COM_IRQn) ;
	#endif

	// Disable STM32F303-only interrupts
	#ifdef STM32F303xC
		HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM1_CC_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM7_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM8_BRK_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM8_CC_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM8_UP_IRQn) ;
		HAL_NVIC_DisableIRQ(TIM8_TRG_COM_IRQn) ;
		HAL_NVIC_DisableIRQ(ADC4_IRQn) ;
		HAL_NVIC_DisableIRQ(UART4_IRQn) ;
		HAL_NVIC_DisableIRQ(UART5_IRQn) ;
	#endif

	HAL_NVIC_DisableIRQ(TIM3_IRQn) ;
	HAL_NVIC_DisableIRQ(TIM4_IRQn) ;
	HAL_NVIC_DisableIRQ(ADC1_2_IRQn) ;
	SysTick->CTRL = 0 ;
}

static void executeApp()
{

	// Disable all peripheral clocks
	// Disable used PLL
	// Disable interrupts
	// Clear pending interrupts

	// Expected at PROGFLASH_START (0x08002000)
	uint32_t *p ;
	p = (uint32_t *) PROGFLASH_START ;

	if ( *p == 0x20005000 )
	{
		USART1->CR1 = 0 ;
		USART1->BRR = 0 ;
		USART2->CR1 = 0 ;
		USART2->BRR = 0 ;
		USART3->CR1 = 0 ;
		USART3->BRR = 0 ;
		#ifdef STM32F103xB
			(void) USART2->SR ;
			(void) USART2->DR ;
			(void) USART1->SR ;
			(void) USART1->DR ;
			USART1->SR = 0 ;
			USART2->SR = 0 ;
			USART3->SR = 0 ;
		#endif
		#ifdef STM32F303xC
			(void) USART2->ISR ;
			(void) USART2->RDR ;
			(void) USART1->ISR ;
			(void) USART1->RDR ;
			USART1->ISR = 0 ;
			USART2->ISR = 0 ;
			USART3->ISR = 0 ;
		#endif

		RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN ;		// Disable clock
		RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN ;		// Disable clock

		// Stop TIM2
		HAL_TIM_Base_Stop(&Timer2Handle);

		disableInterrupts() ;
	
		NVIC->ICER[0] = 0xFFFFFFFF ;
		NVIC->ICER[1] = 0xFFFFFFFF ;
		NVIC->ICER[2] = 0xFFFFFFFF ;
		NVIC->ICPR[0] = 0xFFFFFFFF ;
		NVIC->ICPR[1] = 0xFFFFFFFF ;
		NVIC->ICPR[2] = 0xFFFFFFFF ;
	 
		HAL_RCC_DeInit();  // Use the HAL function
    
		SysTick->CTRL = 0 ;
		SysTick->LOAD = 0 ;
		SysTick->VAL = 0 ;
	
		asm(" mov.w	r1, #134217728");	// 0x8000000
		asm(" add.w	r1, #8192");		// 0x2000

		asm(" movw	r0, #60680");		// 0xED08
		asm(" movt	r0, #57344");		// 0xE000
		asm(" str	r1, [r0, #0]");		// Set the VTOR

		asm("ldr	r0, [r1, #0]");		// Stack pointer value
		asm("msr msp, r0");				// Set it
		asm("ldr	r0, [r1, #4]");		// Reset address
		asm("mov.w	r1, #1");
		asm("orr		r0, r1");		// Set lsbit
		asm("bx r0");					// Execute application
	}
}

// Checks if USART2 has data to be read; reads it
static uint16_t test0()
{
	#ifdef STM32F103xB
		if ( USART2->SR & USART_SR_RXNE )
		{
			return USART2->DR ;
		}
	#endif
	#ifdef STM32F303xC
		if ( USART2->ISR & USART_ISR_RXNE )
		{
			return USART2->RDR ;
		}
	#endif
	return 0xFFFF ;
}

// Checks if USART1 has data to be read; reads it
static uint16_t test1()
{
	#ifdef STM32F103xB
		if ( USART1->SR & USART_SR_RXNE )
		{
			return USART1->DR ;
		}
	#endif
	#ifdef STM32F303xC
		if ( USART1->ISR & USART_ISR_RXNE )
		{
			return USART1->RDR ;
		}
	#endif
	return 0xFFFF ;
}

uint8_t getch1()
{
	#ifdef STM32F103xB
		while ( ( USART1->SR & USART_SR_RXNE ) == 0 )
		{
			if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
			{
				__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);
				__MULTI_TOGGLE_LED();
			}
			// wait
		}
		return USART1->DR ;
	#endif
	#ifdef STM32F303xC
		while ( ( USART1->ISR & USART_ISR_RXNE ) == 0 )
		{
			if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
			{
				__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);
				GPIOA->ODR ^= 0x0002 ;
			}
			// wait
		}
		return USART1->RDR ;
	#endif
}

uint8_t getch()
{
	if ( Port )
	{
		return getch1() ;
	}
	#ifdef STM32F103xB
		while ( ( USART2->SR & USART_SR_RXNE ) == 0 )
		{
			if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
			{
				__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);
				GPIOA->ODR ^= 0x0002 ;
			}
			// wait
		}
		return USART2->DR ;
	#endif
		#ifdef STM32F303xC
		while ( ( USART2->ISR & USART_ISR_RXNE ) == 0 )
		{
			if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
			{
				__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);
				GPIOA->ODR ^= 0x0002 ;
			}
			// wait
		}
		return USART2->RDR ;
	#endif
}

void putch( uint8_t byte )
{
	if ( Port )
	{
		#ifdef STM32F103xB
			while ( ( USART1->SR & USART_SR_TXE ) == 0 )
			{
				// wait
			}
			USART1->DR = byte ;
		#endif
		#ifdef STM32F303xC
			while ( ( USART1->ISR & USART_ISR_TXE ) == 0 )
			{
				// wait
			}
			USART1->RDR = byte ;
		#endif
	}
	else
	{
		#ifdef STM32F103xB
			while ( ( USART3->SR & USART_SR_TXE ) == 0 )
			{
				// wait
			}
			USART3->DR = byte ;
		#endif
		#ifdef STM32F303xC
			while ( ( USART3->ISR & USART_ISR_TXE ) == 0 )
			{
				// wait
			}
			USART3->RDR = byte ;
		#endif
	}
}



void verifySpace()
{
	if (getch() != CRC_EOP)
	{
		NotSynced = 1 ;
		return ;
	}
	putch(STK_INSYNC);
}

void bgetNch(uint8_t count)
{
	do
	{
		getch() ;
	} while (--count) ;
	verifySpace() ;
}

void loader( uint32_t check )
{
	uint8_t ch ;
	uint8_t GPIOR0 ;
	uint32_t address = 0 ;
	uint8_t lastCh ;
	uint8_t serialIsInverted = 0;

	// Skip the BIND button check and stay in the bootloader if reset was requested by software (invoked by the radio)
	if (SoftwareResetReason())
	{
		check = 0 ;	// Stay in bootloader
	}

	HAL_NVIC_DisableIRQ(TIM2_IRQn) ;
	if ( check )
	{
		// Reset TIM2 to 0
		__HAL_TIM_SET_COUNTER(&Timer2Handle, 0);

		// Clear the update flag
		__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);

		// Wait two loops of the timer before checking the BIND button
		uint8_t InterruptCount = 0;
		while (InterruptCount < 2)
		{
			if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
			{
				InterruptCount++;
			}
		}

		// Clear the update flag
		__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);

		// Read the input pins
		ch = GPIOA->IDR & 0xF1 ;

		// Return if the BIND button is not pressed
		if (!CheckForBindButton())
		{
			return ;
		}
	}
	disableInterrupts() ;

	// Unlock the flash
	HAL_FLASH_Unlock();

	NotSynced = 1 ;
	SyncCount = 0;
	lastCh = 0 ;

	for (;;)
	{
		while ( NotSynced )
		{
			uint16_t data ;

			data = test0() ;
			if ( data != 0xFFFF )
			{
				ch = data ;
				if ( ( lastCh == STK_GET_SYNC ) && ( ch == CRC_EOP ) )
				{
					NotSynced = 0 ;
					Port = 0 ;
					break ;
				}
				lastCh = ch ; 
			}

			data = test1() ;
			if ( data != 0xFFFF )
			{
				ch = data ;
				if ( ( lastCh == STK_GET_SYNC ) && ( ch == CRC_EOP ) )
				{
					NotSynced = 0 ;
					Port = 1 ;
					break ;
				}
				lastCh = ch ; 
			}

			if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
			{
				__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);
				GPIOA->ODR ^= 0x0002 ;
			}
		}
		
		/* get character from UART */
		ch = getch() ;

		// Count the number of STK_GET_SYNCs
		if (ch == STK_GET_SYNC)
		{
			SyncCount += 1;
		}
		else
		{
			SyncCount = 0;
		}

		// Toggle serial port inversion if we get five STK_GET_SYNCs in a row
		if (SyncCount > 5)
		{
			if (serialIsInverted == 1)
			{
				DisableSerialInverter();
				serialIsInverted = 0;
			}
			else
			{
				EnableSerialInverter();
				serialIsInverted = 1;
			}
			SyncCount = 0;
		}

		if(ch == STK_GET_PARAMETER)
		{
			GPIOR0 = getch() ;
			verifySpace() ;
			if (GPIOR0 == 0x82)
			{
				putch(OPTIBOOT_MINVER) ;
			}
			else if (GPIOR0 == 0x81)
			{
				putch(OPTIBOOT_MAJVER) ;
			}
			else
			{
				/*
				* GET PARAMETER returns a generic 0x03 reply for
				* other parameters - enough to keep Avrdude happy
				*/
				putch(0x03) ;
			}
		}
		else if(ch == STK_SET_DEVICE)
		{
			// SET DEVICE is ignored
			bgetNch(20) ;
		}
	    else if(ch == STK_SET_DEVICE_EXT)
		{
			// SET DEVICE EXT is ignored
			bgetNch(5);
		}
		else if(ch == STK_LOAD_ADDRESS)
		{
			// LOAD ADDRESS
			uint16_t newAddress ;
			newAddress = getch() ;
			newAddress = (newAddress & 0xff) | (getch() << 8);
			address = newAddress ; // Convert from word address to byte address
			address <<= 1 ;
			verifySpace() ;
		}
		else if(ch == STK_UNIVERSAL)
		{
			// UNIVERSAL command is ignored
			bgetNch(4) ;
			putch(0x00) ;
		}
		else if(ch == STK_PROG_PAGE)
		{
			// PROGRAM PAGE - we support flash programming only, not EEPROM
			uint8_t *bufPtr;
			uint16_t addrPtr;
			uint16_t length ;
			uint16_t count ;
			uint16_t data ;
			uint8_t *memAddress ;
			length = getch() << 8 ;			/* getlen() */
			length |= getch() ;
			getch() ;	// discard flash/eeprom byte
			// While that is going on, read in page contents
			count = length ;
			bufPtr = Buff;
			do
			{
				*bufPtr++ = getch() ;
			}
			while (--count) ;
			if ( length & 1 )
			{
				*bufPtr = 0xFF ;
			}
			count = length ;
			count += 1 ;
			count /= 2 ;
			memAddress = (uint8_t *)(address + 0x08000000) ;

			if ( (uint32_t)memAddress < PROGFLASH_END )
			{
				// Read command terminator, start reply
				verifySpace();

				if ( (uint32_t)memAddress >= PROGFLASH_START )
				{
					if ( ((uint32_t)memAddress & 0x000003FF) == 0 )
					{
						uint32_t SectorError = 0;
						// At page start so erase it
						__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

						EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
						EraseInitStruct.PageAddress = (uint32_t)memAddress;
						EraseInitStruct.NbPages = 1;

						HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
					}
					bufPtr = Buff;
					while ( count )
					{
						data = *bufPtr++ ;
						data |= *bufPtr++ << 8 ;
						HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)memAddress, data);
						memAddress += 2 ;
						count -= 1 ;
					}
				}
			}
			else
			{
				verifySpace();
			}
		}
		else if(ch == STK_READ_PAGE)
	  	{
			uint16_t length ;
			uint8_t xlen ;
			uint8_t *memAddress ;
			memAddress = (uint8_t *)(address + 0x08000000) ;
			// READ PAGE - we only read flash
			xlen = getch() ;			/* getlen() */
			length = getch() | (xlen << 8 ) ;
			getch() ;
		    verifySpace() ;
	    	do
			{
				putch( *memAddress++) ;
			}
	    	while (--length) ;
		}
	    else if(ch == STK_READ_SIGN)
		{
			// READ SIGN - return what Avrdude wants to hear
			verifySpace() ;
			putch(SIGNATURE_0) ;
			if ( Port )
			{
				putch(SIGNATURE_3) ;
				putch(SIGNATURE_4) ;
			}
			else
			{
				putch(SIGNATURE_1) ;
				putch(SIGNATURE_2) ;
			}
		}
		else if (ch == STK_LEAVE_PROGMODE)
		{
			verifySpace() ;
		}
		else
		{
			// This covers the response to commands like STK_ENTER_PROGMODE
			verifySpace() ;
		}
		if ( NotSynced )
		{
			continue ;
		}
		putch(STK_OK);
	}
}

void setup()
{
	// Throw in a startup delay to see if it enables the debugger to see inside this code
	HAL_Delay(5000);

	// Initialize the STM32 HAL
	HAL_Init();

	// Initialize the GPIO pins
	GPIO_Init();

	// Initialize the USARTs
	Serial_Init();

	// Initialize the timer
	Timer_Init();
}

void loop()
{
	// If reset by software, or powered up with the bind button pressed go straight into the bootloader, otherwise run the app

	if (SoftwareResetReason() || CheckForBindButton())
	{
		// Run the loader
		loader(0);
	}

	executeApp();

	// loader(1) ;

	// Execute loaded application
	// executeApp() ;

	// loader(0) ;
	
	// If we get here it's because we didn't go into the bootloader and the application code didn't run when we tried to launch it
	// Blink the LED forever...
	for(;;)
	{
		if (__HAL_TIM_GET_FLAG(&Timer2Handle, TIM_FLAG_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&Timer2Handle, TIM_IT_UPDATE);
			if ( ++LongCount > 4 )
			{
				// GPIOA->ODR ^= 0x0002 ;
				__MULTI_TOGGLE_LED();
				LongCount = 0 ;
			}
		}
	}
}
