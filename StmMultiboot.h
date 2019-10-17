#ifndef __STMMULTIBOOT_H
#define __STMMULTIBOOT_H

#if (!defined(STM32F103xB) && !defined(ARDUINO_BLUEPILL_F103C8)) && (!defined(STM32F303xC) && !defined(ARDUINO_BLACKPILL_F303CC))
  #error "Board must be 'Generic STM32F1 series -> BluePill F103C8 (128K)' or 'Generic STM32F3 series -> RobotDyn BlackPill F303CC'"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #error "Serial must be disabled in the Tools -> U(S)ART Support menu (in order to keep the binary under 8KB)."
#endif

#endif /* __STMMULTIBOOT_H */
