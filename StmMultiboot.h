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

#ifndef __STMMULTIBOOT_H
#define __STMMULTIBOOT_H

// Verify that the correct board is selected
#if (!defined(STM32F103xB) && !defined(ARDUINO_BLUEPILL_F103C8)) && (!defined(STM32F303xC) && !defined(ARDUINO_BLACKPILL_F303CC))
#error "Board must be 'Generic STM32F1 series -> BluePill F103C8 (128K)' or 'Generic STM32F3 series -> RobotDyn BlackPill F303CC'"
#endif

// Verify that seiral support is disabled in the board menu
#ifdef HAL_UART_MODULE_ENABLED
#error "Serial must be disabled in the Tools -> U(S)ART Support menu (in order to keep the binary under 8KB)."
#endif

// Bytes for the device signature we return to the radio - 0x1E55AA
#define SIGNATURE_0		0x1E
#define SIGNATURE_1		0x55
#define SIGNATURE_2		0xAA

// Version numbers
#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 7

// Value to place in RTC backup register 10 for persistent bootloader mode
#define RTC_BOOTLOADER_FLAG				0x424C
#define RTC_BOOTLOADER_JUST_UPLOADED	0x424D
#define RTC_BOOTLOADER_APP_RUNNING		0x4252

// Boundaries of program flash space, EEPROM space, and RAM - varies by MCU
#define FLASH_START (uint32_t)0x08000000
#define PROGFLASH_START (uint32_t)0x08002000
#ifdef STM32F103xB
#define EEPROM_START (uint32_t)0x0801F800
#define RAM_SIZE (uint32_t)0x00005000
#endif
#ifdef STM32F303xC
#define EEPROM_START (uint32_t)0x0803F800
#define RAM_SIZE (uint32_t)0x0000A000
#endif
#define PROGFLASH_SIZE = EEPROM_START - PROGFLASH_START - 1

void DisableInterrupts();
void BackupRegisterWrite(uint32_t BackupRegister, uint32_t data);

#endif /* __STMMULTIBOOT_H */
