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

/* Toggles the hardware serial port inverter - inversion only applies to the serial TX pin */
static void ToggleSerialInverter()
{
	// Toggle PB3
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

/*
 * Checks if USART2 has data to be read
 * If there is data, read and return it; if not, return 0xFFFF
 */
uint16_t TestUsart()
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

	// Disable the interrupts
	DisableInterrupts();

	notSynced = 1;
	syncCount = 0;
	lastCh = 0;

	uint32_t addressOffset = 0;

	for (;; )
	{
		while (notSynced)
		{
			uint16_t data;

			// Check for serial data on USART2
			data = TestUsart();
			if (data != 0xFFFF)
			{
				ch = data;
				if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
				{
					notSynced = 0;
					break;
				}
				lastCh = ch;
			}
		}

		// Unlock the flash
		HAL_FLASH_Unlock();

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
		// Repeating SYNCs indicates we are receiving serial data from the radio but the radio is not getting our responses, 
		// so try inverting the TX data to see if that's what the radio expects
		if (syncCount > 4)
		{
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
			PutChar(SIGNATURE_1);
			PutChar(SIGNATURE_2);
		}
		else if (ch == STK_ENTER_PROGMODE)
		{
			VerifyCommand();

		}
		else if (ch == STK_LEAVE_PROGMODE)
		{
			VerifyCommand();

			// Lock the flash
			HAL_FLASH_Lock();

			// Set the backup register flag to say firmware was just uploaded
			// BackupRegisterWrite(RTC_BKP_DR10, RTC_BOOTLOADER_JUST_UPLOADED);
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

			// If address is 0 move it up to the start of program flash space - corrects the initial offset for uploads from AVRDUDE
			if (address == 0)
			{
				addressOffset = (uint32_t)(PROGFLASH_START - FLASH_START);
			}

			// Offset the write by the flash start address
			memAddress = (uint8_t *)(address + FLASH_START + addressOffset);

			// Only write to addresses that are above the bootloader and below the EEPROM
			if ((uint32_t)memAddress >= PROGFLASH_START && (uint32_t)memAddress < EEPROM_START)
			{
				// Read command terminator, start reply
				VerifyCommand();

				// Check if this is the start of the page; if so we'll erase it
				if (((uint32_t)memAddress & 0x000003FF) == 0)
				{
					// Clear the flash flags
					__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

					// Structure for erasing flash pages
					FLASH_EraseInitTypeDef EraseInitStruct;
					uint32_t SectorError = 0;

					// Configure the erase
					EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
					EraseInitStruct.PageAddress = (uint32_t)memAddress;
					EraseInitStruct.NbPages = 1;

					// Do the erase
					HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
				}

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

			// If address is 0 move it up to the start of program flash space - corrects the initial offset for uploads from AVRDUDE
			if (address == 0)
			{
				addressOffset = (uint32_t)(PROGFLASH_START - FLASH_START);
			}

			// Offset the read by the program flash start address
			memAddress = (uint8_t *)(address + FLASH_START + addressOffset);

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
