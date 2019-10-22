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

// Buffer for serial read/write operations
uint8_t serialBuffer[512];

// Flag to indicate STK sync status
uint8_t notSynced;

// Counter for STK SYNC packets
uint8_t syncCount;

// Flag to indicate the active serial port(s); 0 = USART2 + USART3; 1 = USART1
uint8_t usart1Active;

/* 
 * Timer 2 interrupt handler override
 * Update interrupts are used to flash the red LED when the bootloader is active.
 */
extern "C" {
	void TIM2_IRQHandler(void) {
		// Handle update interrupts (update interrupt flag is set)
		if (TIM2->SR & TIM_SR_UIF) {
			// Reset the update interrupt flag
			TIM2->SR &= ~(TIM_SR_UIF);
			
			// Toggle the LED
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
		}
	}
}

/*
 * Initializes Timer 2
 * Configures TIM2 without using the HAL functions so that we can redefine the IRQ handler and keep the bootloader size down.
 * Frequency is 15Hz: 72000000 / ((479+1) * (9999+1)) = 15
 */
static void Timer_Init()
{
	__HAL_RCC_TIM2_CLK_ENABLE();	// Enable the clock
	TIM2->CNT = 0;					// Zero the count
	TIM2->PSC = 479;				// Prescaler
	TIM2->ARR = 9999;				// Period
	TIM2->DIER = TIM_DIER_UIE;		// Update interrupt enable
	TIM2->CR1 |= TIM_CR1_CEN;		// Enable the timer
	HAL_NVIC_EnableIRQ(TIM2_IRQn);	// Enable interrupts from TIM2
}

/* Initializes the GPIO pins for inputs, outputs, and USARTs */
static void GPIO_Init()
{
	// Structure for configuring GPIO pins
	GPIO_InitTypeDef GPIO_InitStruct;

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

	// Configure PA1 and PA2 as outputs (red LED is PA1; green LED is PA2)
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PB1 and PB3 as output (enable/disable serial inverter)
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Start with the serial inverter disabled - set PB1 and clear PB3 
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	// Configure PA3 as alternate function USART2_RX (USART2_TX=PA2, USART2_RX=PA3 - only RX (PA3) is used)
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
#ifdef STM32F303xC
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
#endif
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PB10 as alternate function USART3_TX (USART3_TX=PB10, USART3_RX=PB11 - only TX (PB10) is used)
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
#ifdef STM32F303xC
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
#endif
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if defined(STM32F103xB) && !defined(_DEBUG)
	__HAL_AFIO_REMAP_SWJ_DISABLE();		// Disable JTAG and SWD
#endif
}

/* Initializes the serial ports */
static void Serial_Init()
{
	// Enable the clocks
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();

	// Set the baud rates and configure the ports
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
}

/* Disables the hardware serial port inverter */
/*
static void DisableSerialInverter()
{
	// Set PB1 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// Clear PB3 (LOW)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}
*/

/* Enables the hardware serial port inverter */
/*
static void EnableSerialInverter()
{
	// Set PB1 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// Set PB3 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}
*/

/* Toggles the hardware serial port inverter */
static void ToggleSerialInverter()
{
	// Toggle PB3
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

/* 
 * Checks the state of the BIND button and the rotary selector
 * Returns 1 if the BIND button is pressed and the rotary selector is on '0', otherwise returns 0.
 */
static uint32_t CheckForBindButton()
{
	return ((GPIOA->IDR & 0xF1) != 0xF0) ? 0 : 1;
}

/* Returns 1 if the device was reset via a software request, 0 for any other reset reason */
static uint32_t SoftwareResetReason()
{
	// Clear the reset flag
	RCC->CSR |= RCC_CSR_RMVF;

	// Return 1 for a software reset, otherwise 0
	return (RCC->CSR & RCC_CSR_SFTRSTF) ? 1 : 0;
}

/* Disables interrupts */
void DisableInterrupts()
{
	//__disable_irq();
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	HAL_NVIC_DisableIRQ(ADC1_2_IRQn);

#ifdef STM32F103xB
	HAL_NVIC_DisableIRQ(TIM1_BRK_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
#endif
#ifdef STM32F303xC
	HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
	HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
	HAL_NVIC_DisableIRQ(TIM7_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_BRK_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_UP_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_TRG_COM_IRQn);
	HAL_NVIC_DisableIRQ(ADC4_IRQn);
	HAL_NVIC_DisableIRQ(UART4_IRQn);
	HAL_NVIC_DisableIRQ(UART5_IRQn);
#endif
	SysTick->CTRL = 0;
}

/* Checks for a valid pointer at the beginning of the application flash space */
uint8_t CheckForApplication(void)
{
	return (((*(__IO uint32_t*)PROGFLASH_START) & ~(RAM_SIZE)) == 0x20000000) ? 1 : 0;
}

/* Jumps to the application */
void JumpToApplication(void)
{
	typedef void(*pFunction)(void);

	// Jump address is stored four bytes in from the start of the program flash space
	uint32_t jumpAddress = *(__IO uint32_t*)(PROGFLASH_START + 4);
	pFunction Jump = (pFunction)jumpAddress;

	// Disable the interrupts
	DisableInterrupts();

	// Disable the clocks
	RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;

	// Disable the timer
	TIM2->CR1 &= ~TIM_CR1_CEN;

	// Clear any interrupts
	NVIC->ICER[0] = 0xFFFFFFFF;
	NVIC->ICER[1] = 0xFFFFFFFF;
	NVIC->ICER[2] = 0xFFFFFFFF;
	NVIC->ICPR[0] = 0xFFFFFFFF;
	NVIC->ICPR[1] = 0xFFFFFFFF;
	NVIC->ICPR[2] = 0xFFFFFFFF;

	//HAL_RCC_DeInit();
	HAL_DeInit();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	// Update the vector table
	SCB->VTOR = PROGFLASH_START;

	// Jump to the application code
	//__set_MSP(*(__IO uint32_t*)PROGFLASH_START);
	Jump();
}

/*
 * Checks if USART1 has data to be read
 * If there is data, read it; if not, return 0xFFFF
 */
static uint16_t TestUsart1()
{
#ifdef STM32F103xB
	if (USART1->SR & USART_SR_RXNE)
	{
		return USART1->DR;
	}
#endif
#ifdef STM32F303xC
	if (USART1->ISR & USART_ISR_RXNE)
	{
		return USART1->RDR;
	}
#endif
	return 0xFFFF;
}

/* 
 * Checks if USART2 has data to be read
 * If there is data, read it; if not, return 0xFFFF
 */
static uint16_t TestUsart2()
{
#ifdef STM32F103xB
	if (USART2->SR & USART_SR_RXNE)
	{
		return USART2->DR;
	}
#endif
#ifdef STM32F303xC
	if (USART2->ISR & USART_ISR_RXNE)
	{
		return USART2->RDR;
	}
#endif
	return 0xFFFF;
}

/* Gets a character from USART1 */
uint8_t GetChar_1()
{
#ifdef STM32F103xB
	while ((USART1->SR & USART_SR_RXNE) == 0)
	{
		// wait
	}
	return USART1->DR;
#endif
#ifdef STM32F303xC
	while ((USART1->ISR & USART_ISR_RXNE) == 0)
	{
		// wait
	}
	return USART1->RDR;
#endif
}

/* Gets a character from serial */
uint8_t GetChar()
{
	if (usart1Active)
	{
		return GetChar_1();
	}
#ifdef STM32F103xB
	while ((USART2->SR & USART_SR_RXNE) == 0)
	{
		// wait
	}
	return USART2->DR;
#endif
#ifdef STM32F303xC
	while ((USART2->ISR & USART_ISR_RXNE) == 0)
	{
		// wait
	}
	return USART2->RDR;
#endif
}

/* Sends a character to the serial device */
void PutChar(uint8_t byte)
{
	if (usart1Active)
	{
#ifdef STM32F103xB
		while ((USART1->SR & USART_SR_TXE) == 0)
		{
			// wait
		}
		USART1->DR = byte;
#endif
#ifdef STM32F303xC
		while ((USART1->ISR & USART_ISR_TXE) == 0)
		{
			// wait
		}
		USART1->RDR = byte;
#endif
	}
	else
	{
#ifdef STM32F103xB
		while ((USART3->SR & USART_SR_TXE) == 0)
		{
			// wait
		}
		USART3->DR = byte;
#endif
#ifdef STM32F303xC
		while ((USART3->ISR & USART_ISR_TXE) == 0)
		{
			// wait
		}
		USART3->RDR = byte;
#endif
	}
}

/* 
 * Verifies that the next character received after an STK command is a CRC_EOP (SPACE)
 * Sends STK_INSYNC if it is, otherwise sets NotSynced
 */
void VerifyCommand()
{
	if (GetChar() != CRC_EOP)
	{
		notSynced = 1;
		return;
	}
	PutChar(STK_INSYNC);
}

/*
 * Discards STK commands we don't care about
 * Skips <count> characters
 */
void SkipChars(uint8_t count)
{
	do
	{
		GetChar();
	} while (--count);

	VerifyCommand();
}

// Main bootloader routine
void FlashLoader()
{
	// Character to read from or write to serial
	uint8_t ch;

	// Flash address to write to or read from
	uint32_t address = 0;

	// Last character from serial - used when checking for SYNC
	uint8_t lastCh;
	
	// Indicate if serial port is currently inverted
	// uint8_t serialIsInverted = 0;

	// Disable the interrupts
	DisableInterrupts();

	notSynced = 1;
	syncCount = 0;
	lastCh = 0;

	for (;; )
	{
		while (notSynced)
		{
			uint16_t data;

			// Check for serial data on USART2
			data = TestUsart2();
			if (data != 0xFFFF)
			{
				ch = data;
				if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
				{
					notSynced = 0;
					usart1Active = 0;
					break;
				}
				lastCh = ch;
			}

			// Check for serial data on USART1
			data = TestUsart1();
			if (data != 0xFFFF)
			{
				ch = data;
				if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
				{
					notSynced = 0;
					usart1Active = 1;
					break;
				}
				lastCh = ch;
			}
		}

		/* Get character from UART */
		ch = GetChar();

		// Count the number of STK_GET_SYNCs
		if (ch == STK_GET_SYNC)
		{
			syncCount += 1;
		}
		else
		{
			syncCount = 0;
		}

		// Toggle serial port inversion if we get five STK_GET_SYNCs in a row
		if (syncCount > 4)
		{
			/*
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
			*/
			ToggleSerialInverter();
			syncCount = 0;
		}

		if (ch == STK_GET_PARAMETER)
		{
			uint8_t cmdParameter = GetChar();
			VerifyCommand();
			if (cmdParameter == 0x82)
			{
				PutChar(OPTIBOOT_MINVER);
			}
			else if (cmdParameter == 0x81)
			{
				PutChar(OPTIBOOT_MAJVER);
			}
			else
			{
				// Return a generic 0x03 reply to keep AVRDUDE happy
				PutChar(0x03);
			}
		}
		else if (ch == STK_SET_DEVICE)
		{
			// SET DEVICE is ignored
			SkipChars(20);
		}
		else if (ch == STK_SET_DEVICE_EXT)
		{
			// SET DEVICE EXT is ignored
			SkipChars(5);
		}
		else if (ch == STK_UNIVERSAL)
		{
			// UNIVERSAL command is ignored
			SkipChars(4);
			PutChar(0x00);
		}
		else if (ch == STK_LOAD_ADDRESS)
		{
			// LOAD ADDRESS
			uint16_t newAddress;
			newAddress = GetChar();
			newAddress = (newAddress & 0xff) | (GetChar() << 8);
			address = newAddress; // Convert from word address to byte address
			address <<= 1;
			VerifyCommand();
		}
		else if (ch == STK_READ_SIGN)
		{
			// Return the signature
			VerifyCommand();
			PutChar(SIGNATURE_0);
			if (usart1Active)
			{
				PutChar(SIGNATURE_3);
				PutChar(SIGNATURE_4);
			}
			else
			{
				PutChar(SIGNATURE_1);
				PutChar(SIGNATURE_2);
			}
		}
		else if (ch == STK_ENTER_PROGMODE)
		{
			VerifyCommand();

			// Unlock the flash
			HAL_FLASH_Unlock();

			// Structure for erasing flash pages
			FLASH_EraseInitTypeDef EraseInitStruct;
			uint32_t SectorError = 0;
			
			// Clear the flash flags
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

			// Configure the erase
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;											// Erase pages (not a mass erase)
			EraseInitStruct.PageAddress = (uint32_t)PROGFLASH_START;									// Start erase at the first page of program flash (preserve the bootloader and EEPROM pages)
			EraseInitStruct.NbPages = uint32_t((EEPROM_START - PROGFLASH_START) / FLASH_PAGE_SIZE);		// Erase up to the EEPROM pages

			// Do the erase
			HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
		}
		else if (ch == STK_LEAVE_PROGMODE)
		{
			VerifyCommand();

			// Lock the flash
			HAL_FLASH_Lock();
		}
		else if (ch == STK_PROG_PAGE)
		{
			// PROGRAM PAGE - we support flash programming only, not EEPROM
			uint8_t *bufPtr;
			uint16_t length;
			uint16_t count;
			uint16_t data;
			uint8_t *memAddress;
			length = GetChar() << 8;
			length |= GetChar();
			GetChar();	// discard flash/eeprom byte
			// While that is going on, read in page contents
			count = length;
			bufPtr = serialBuffer;
			do
			{
				*bufPtr++ = GetChar();
			} while (--count);
			if (length & 1)
			{
				*bufPtr = 0xFF;
			}
			count = length;
			count += 1;
			count /= 2;

			// Offset the write by the program flash start address
			memAddress = (uint8_t *)(address + PROGFLASH_START);

			// Only write to addresses that are above the bootloader and below the EEPROM
			if ((uint32_t)memAddress >= PROGFLASH_START && (uint32_t)memAddress < EEPROM_START)
			{
				// Read command terminator, start reply
				VerifyCommand();

				bufPtr = serialBuffer;
				while (count)
				{
					data = *bufPtr++;
					data |= *bufPtr++ << 8;
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)memAddress, data);
					memAddress += 2;
					count -= 1;
				}
			}
			else
			{
				VerifyCommand();
			}
		}
		else if (ch == STK_READ_PAGE)
		{
			uint16_t length;
			uint8_t xlen;
			uint8_t *memAddress;

			// Offset the read by the program flash start address
			memAddress = (uint8_t *)(address + PROGFLASH_START);

			xlen = GetChar();
			length = GetChar() | (xlen << 8);
			GetChar();
			VerifyCommand();
			do
			{
				PutChar(*memAddress++);
			} while (--length);
		}
		else
		{
			VerifyCommand();
		}
		if (notSynced)
		{
			continue;
		}

		PutChar(STK_OK);
	}
}

void setup()
{
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
	// If reset by software, or powered up with protocol 0 and the bind button pressed, or there's not application, go straight into the bootloader, otherwise run the app
	if (SoftwareResetReason() || CheckForBindButton() || !CheckForApplication())
	{
		// Run the main bootloader routine
		FlashLoader();
	}

	// Launch the Multi firmware if there's a valid application to launch
	if (CheckForApplication())
	{
		// Jump to the application code at PROGFLASH_START
		JumpToApplication();
	}
}
