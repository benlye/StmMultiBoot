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
uint8_t Buff[512];

// Flag to indicate STK sync status
uint8_t NotSynced;

// Counter for STK SYNC packets
uint8_t SyncCount;

// Flag to indicate active serial port(s); 0 = USART2 + USART3; 1 = USART1
uint8_t Port;

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

	// Configure PA1 and PA2 as outputs (red and green LEDs)
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

	// Configure PA9 as alternate function USART2_RX (USART2_TX=PA2, USART2_RX=PA9 - only RX (PA9) is used)
	GPIO_InitStruct.Pin = GPIO_PIN_9;
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

	// Disable JTAG
	//__HAL_AFIO_REMAP_SWJ_DISABLE();		// Disable JTAG and SWD - use for released version
	__HAL_AFIO_REMAP_SWJ_NOJTAG();			// Disable JTAG but keep SWD enabled - use for development/debugging
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

/* Toggles the hardware serial port inverter */
static void ToggleSerialInverter()
{
	// Toggle PB3
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

/* Checks the state of the BIND button */
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

	// Disable STM32F103-only interrupts
#ifdef STM32F103xB
	HAL_NVIC_DisableIRQ(TIM1_BRK_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
	HAL_NVIC_DisableIRQ(TIM1_TRG_COM_IRQn);
#endif

	// Disable STM32F303-only interrupts
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

	HAL_NVIC_DisableIRQ(TIM3_IRQn);
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
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
	uint32_t  JumpAddress = *(__IO uint32_t*)(PROGFLASH_START + 4);
	pFunction Jump = (pFunction)JumpAddress;

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

/* Checks if USART2 has data to be read; reads it */
static uint16_t test0()
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

/* Checks if USART1 has data to be read; reads it */
static uint16_t test1()
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

uint8_t getch1()
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

uint8_t getch()
{
	if (Port)
	{
		return getch1();
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

void putch(uint8_t byte)
{
	if (Port)
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

void verifySpace()
{
	if (getch() != CRC_EOP)
	{
		NotSynced = 1;
		return;
	}
	putch(STK_INSYNC);
}

void bgetNch(uint8_t count)
{
	do
	{
		getch();
	} while (--count);
	verifySpace();
}

// Main bootloader routine
void FlashLoader()
{
	uint8_t ch;
	uint8_t GPIOR0;
	uint32_t address = 0;
	uint8_t lastCh;
	uint8_t serialIsInverted = 0;

	// Disable the interrupts
	DisableInterrupts();

	NotSynced = 1;
	SyncCount = 0;
	lastCh = 0;

	for (;; )
	{
		while (NotSynced)
		{
			uint16_t data;

			data = test0();
			if (data != 0xFFFF)
			{
				ch = data;
				if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
				{
					NotSynced = 0;
					Port = 0;
					break;
				}
				lastCh = ch;
			}

			data = test1();
			if (data != 0xFFFF)
			{
				ch = data;
				if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
				{
					NotSynced = 0;
					Port = 1;
					break;
				}
				lastCh = ch;
			}
		}

		/* get character from UART */
		ch = getch();

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

		if (ch == STK_GET_PARAMETER)
		{
			GPIOR0 = getch();
			verifySpace();
			if (GPIOR0 == 0x82)
			{
				putch(OPTIBOOT_MINVER);
			}
			else if (GPIOR0 == 0x81)
			{
				putch(OPTIBOOT_MAJVER);
			}
			else
			{
				// Return a generic 0x03 reply to keep AVRDUDE happy
				putch(0x03);
			}
		}
		else if (ch == STK_SET_DEVICE)
		{
			// SET DEVICE is ignored
			bgetNch(20);
		}
		else if (ch == STK_SET_DEVICE_EXT)
		{
			// SET DEVICE EXT is ignored
			bgetNch(5);
		}
		else if (ch == STK_UNIVERSAL)
		{
			// UNIVERSAL command is ignored
			bgetNch(4);
			putch(0x00);
		}
		else if (ch == STK_LOAD_ADDRESS)
		{
			// LOAD ADDRESS
			uint16_t newAddress;
			newAddress = getch();
			newAddress = (newAddress & 0xff) | (getch() << 8);
			address = newAddress; // Convert from word address to byte address
			address <<= 1;
			verifySpace();
		}
		else if (ch == STK_READ_SIGN)
		{
			// Return the signature
			verifySpace();
			putch(SIGNATURE_0);
			if (Port)
			{
				putch(SIGNATURE_3);
				putch(SIGNATURE_4);
			}
			else
			{
				putch(SIGNATURE_1);
				putch(SIGNATURE_2);
			}
		}
		else if (ch == STK_ENTER_PROGMODE)
		{
			verifySpace();

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
			verifySpace();

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
			length = getch() << 8;
			length |= getch();
			getch();	// discard flash/eeprom byte
			// While that is going on, read in page contents
			count = length;
			bufPtr = Buff;
			do
			{
				*bufPtr++ = getch();
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
				verifySpace();

				bufPtr = Buff;
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
				verifySpace();
			}
		}
		else if (ch == STK_READ_PAGE)
		{
			uint16_t length;
			uint8_t xlen;
			uint8_t *memAddress;

			// Offset the read by the program flash start address
			memAddress = (uint8_t *)(address + PROGFLASH_START);

			xlen = getch();
			length = getch() | (xlen << 8);
			getch();
			verifySpace();
			do
			{
				putch(*memAddress++);
			} while (--length);
		}
		else
		{
			verifySpace();
		}
		if (NotSynced)
		{
			continue;
		}
		putch(STK_OK);
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
		JumpToApplication();
	}
}
