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

#if (!defined(STM32F103xB) && !defined(ARDUINO_BLUEPILL_F103C8)) && (!defined(STM32F303xC) && !defined(ARDUINO_BLACKPILL_F303CC))
  #error "Board must be 'Generic STM32F1 series -> BluePill F103C8 (128K)' or 'Generic STM32F3 series -> RobotDyn BlackPill F303CC'"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #error "Serial must be disabled in the Tools -> U(S)ART Support menu (in order to keep the binary under 8KB)."
#endif

#endif /* __STMMULTIBOOT_H */
