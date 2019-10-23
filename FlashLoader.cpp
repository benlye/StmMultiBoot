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

#include "FlashLoader.h"

// Buffer for serial read/write operations
uint8_t serialBuffer[512];

// Flag to indicate STK sync status
uint8_t notSynced;

// Counter for STK SYNC packets
uint8_t syncCount;

// Flag to indicate the active serial port(s); 0 = USART2 + USART3; 1 = USART1
uint8_t usart1Active;

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
 * Checks if USART1 has data to be read
 * If there is data, read and return it; if not, return 0xFFFF
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
 * If there is data, read and return it; if not, return 0xFFFF
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

/* Gets a character from serial */
uint8_t GetChar()
{
	USART_TypeDef* USART;

	if (usart1Active)
	{
		USART = USART1;
	}
	else
	{
		USART = USART2;
	}
#ifdef STM32F103xB
	while ((USART->SR & USART_SR_RXNE) == 0)
	{
		// wait
	}
	return USART->DR;
#endif
#ifdef STM32F303xC
	while ((USART->ISR & USART_ISR_RXNE) == 0)
	{
		// wait
	}
	return USART->RDR;
#endif
}

/* Sends a character to the serial device */
void PutChar(uint8_t byte)
{
	USART_TypeDef* USART;

	if (usart1Active)
	{
		USART = USART1;
	}
	else
	{
		USART = USART3;
	}
#ifdef STM32F103xB
	while ((USART->SR & USART_SR_TXE) == 0)
	{
		// wait
	}
	USART->DR = byte;
#endif
#ifdef STM32F303xC
	while ((USART->ISR & USART_ISR_TXE) == 0)
	{
		// wait
	}
	USART->RDR = byte;
#endif
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
		if (syncCount > 5)
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


