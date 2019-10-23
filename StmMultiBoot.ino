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
#include "FlashLoader.h"

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

	// Configure PA9 as alternate function for USART1 (USART1_TX=PA9, USART1_RX=PA10 )
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
#ifdef STM32F303xC
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
#endif
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PA10 as alternate function for USART1 (USART1_TX=PA9, USART1_RX=PA10 )
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
#ifdef STM32F303xC
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
#endif
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PA3 as alternate function USART2_RX (USART2_TX=PA2, USART2_RX=PA3 - only RX (PA3) is used)
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
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
