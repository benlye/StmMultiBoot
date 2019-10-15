#include "stk500.h"

#define _FLASH_PROG		1

// Temp
#define SIGNATURE_0		0x1E
#define SIGNATURE_1		0x55 //0x97
#define SIGNATURE_2		0xAA //0x02
#define SIGNATURE_3		0x97
#define SIGNATURE_4		0x02

#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 5

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
uint8_t Port ;

static void start_timer2()
{	
	TIM2->CNT = 0 ;
	TIM2->PSC = 71 ;			// 72-1;for 72 MHZ /1.0usec/(71+1)
	TIM2->ARR = 0xFFFF;		//count till max
}

void RCC_DeInit(void)
{
	/* Disable APB2 Peripheral Reset */
	RCC->APB2RSTR = 0x00000000;

	/* Disable APB1 Peripheral Reset */
	RCC->APB1RSTR = 0x00000000;

	/* FLITF and SRAM Clock ON */
	RCC->AHBENR = 0x00000014;

	/* Disable APB2 Peripheral Clock */
	RCC->APB2ENR = 0x00000000;

	/* Disable APB1 Peripheral Clock */
	RCC->APB1ENR = 0x00000000;

	/* Set HSION bit */
	RCC->CR |= (uint32_t)0x00000001;

	/* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits*/
	RCC->CFGR &= 0xF8FF0000;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= 0xFEF6FFFF;

	/* Reset HSEBYP bit */
	RCC->CR &= 0xFFFBFFFF;

	/* Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits */
	RCC->CFGR &= 0xFF80FFFF;

	/* Disable all interrupts */
	RCC->CIR = 0x009F0000;
}

void disableInterrupts()
{
	__disable_irq() ;
	HAL_NVIC_DisableIRQ(USART1_IRQn) ;
	HAL_NVIC_DisableIRQ(USART2_IRQn) ;
	HAL_NVIC_DisableIRQ(USART3_IRQn) ;

	// Disable STM32F103-only interrupts
	#ifdef STM32F103xB
		NVIC_DisableIRQ(TIM1_BRK_IRQn) ;
		NVIC_DisableIRQ(TIM1_CC_IRQn) ;
		NVIC_DisableIRQ(TIM1_UP_IRQn) ;
		NVIC_DisableIRQ(TIM1_TRG_COM_IRQn) ;
	#endif

	// Disable STM32F303-only interrupts
	#ifdef STM32F303xC
		NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn) ;
		NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn) ;
		NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn) ;
		NVIC_DisableIRQ(TIM1_CC_IRQn) ;
		NVIC_DisableIRQ(TIM6_DAC_IRQn) ;
		NVIC_DisableIRQ(TIM7_IRQn) ;
		NVIC_DisableIRQ(TIM8_BRK_IRQn) ;
		NVIC_DisableIRQ(TIM8_CC_IRQn) ;
		NVIC_DisableIRQ(TIM8_UP_IRQn) ;
		NVIC_DisableIRQ(TIM8_TRG_COM_IRQn) ;
		NVIC_DisableIRQ(ADC4_IRQn) ;
		NVIC_DisableIRQ(UART4_IRQn) ;
		NVIC_DisableIRQ(UART5_IRQn) ;
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

		TIM2->CR1 = 0 ;

		disableInterrupts() ;
	
		NVIC->ICER[0] = 0xFFFFFFFF ;
		NVIC->ICER[1] = 0xFFFFFFFF ;
		NVIC->ICER[2] = 0xFFFFFFFF ;
		NVIC->ICPR[0] = 0xFFFFFFFF ;
		NVIC->ICPR[1] = 0xFFFFFFFF ;
		NVIC->ICPR[2] = 0xFFFFFFFF ;
	 
		RCC_DeInit() ;
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
			IWDG->KR = 0xAAAA ;		// reload
			if ( TIM2->SR & TIM_SR_UIF )
			{
			TIM2->SR &= ~TIM_SR_UIF ;
				GPIOA->ODR ^= 0x0002 ;
			}
			// wait
		}
		return USART1->DR ;
	#endif
	#ifdef STM32F303xC
		while ( ( USART1->ISR & USART_ISR_RXNE ) == 0 )
		{
			IWDG->KR = 0xAAAA ;		// reload
			if ( TIM2->SR & TIM_SR_UIF )
			{
			TIM2->SR &= ~TIM_SR_UIF ;
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
			IWDG->KR = 0xAAAA ;		// reload
			if ( TIM2->SR & TIM_SR_UIF )
			{
			TIM2->SR &= ~TIM_SR_UIF ;
				GPIOA->ODR ^= 0x0002 ;
			}
			// wait
		}
		return USART2->DR ;
	#endif
		#ifdef STM32F303xC
		while ( ( USART2->ISR & USART_ISR_RXNE ) == 0 )
		{
			IWDG->KR = 0xAAAA ;		// reload
			if ( TIM2->SR & TIM_SR_UIF )
			{
			TIM2->SR &= ~TIM_SR_UIF ;
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

static void serialInit()
{
	__HAL_RCC_USART1_CLK_ENABLE();		// Enable clock
	__HAL_RCC_USART2_CLK_ENABLE();		// Enable clock
	__HAL_RCC_USART3_CLK_ENABLE();		// Enable clock

	__HAL_RCC_GPIOA_CLK_ENABLE() ;
	__HAL_RCC_GPIOB_CLK_ENABLE() ;

	#ifdef __HAL_RCC_AFIO_CLK_ENABLE
		__HAL_RCC_AFIO_CLK_ENABLE() ;
	#endif

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

	USART1->BRR = 72000000 / 57600 ;
	USART1->CR1 = 0x200C ;
	USART2->BRR = 36000000 / 57600 ;
	USART2->CR1 = 0x200C ;
	USART2->CR2 = 0 ;
	USART2->CR3 = 0 ;
	USART3->BRR = 36000000 / 57600 ;
	USART3->CR1 = 0x200C ;
	USART3->CR2 = 0 ;
	USART3->CR3 = 0 ;

}

void setup()
{
	// Initialize the STM32 HAL
	HAL_Init();

	// Initialize the USARTs
	serialInit() ;

	// Start the timer used to blink the LED rapidly
	start_timer2() ; //0.5us

	// Unlock the flash
	HAL_FLASH_Unlock() ;

	// Set PA0, 4-7 (HIGH)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

	// Configure pins PA0, 4-7 as inputs
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configure PA1 as output (LED)
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// Configure PB1 and PB3 as output
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Clear PB3 (LOW)
	GPIOB->BRR = 0x00000008 ; // 0b1000
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	
	// Set PB1 (HIGH)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
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

	ResetReason = RCC->CSR ;
	RCC->CSR |= RCC_CSR_RMVF ;
	if ( ResetReason & RCC_CSR_SFTRSTF )
	{
		check = 0 ;	// Stay in bootloader
	}

	NVIC_DisableIRQ(TIM2_IRQn) ;
	if ( check )
	{
		TIM2->CNT = 0 ;
		TIM2->SR &= ~TIM_SR_UIF ;
		while ( ( TIM2->SR & TIM_SR_UIF ) == 0 )
		{
			// wait
		}
		TIM2->SR &= ~TIM_SR_UIF ;

		while ( ( TIM2->SR & TIM_SR_UIF ) == 0 )
		{
			// wait
		}
		TIM2->SR &= ~TIM_SR_UIF ;

		ch = GPIOA->IDR & 0xF1 ;
		if ( ch != 0xF0 )
		{
			return ;
		}
	}
	disableInterrupts() ;

	NotSynced = 1 ;
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

			IWDG->KR = 0xAAAA ;		// reload

			if ( TIM2->SR & TIM_SR_UIF )
			{
	  			TIM2->SR &= ~TIM_SR_UIF ;
				GPIOA->ODR ^= 0x0002 ;
			}
		}
		
		/* get character from UART */
		ch = getch() ;
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

void loop()
{
	loader(1) ;

	// Execute loaded application
	executeApp() ;

	loader(0) ;
	
	// The next bit not really needed as loader(0) doesn't return	
	for(;;)
	{
		if ( TIM2->SR & TIM_SR_UIF )
		{
			TIM2->SR &= ~TIM_SR_UIF ;
			if ( ++LongCount > 4 )
			{
				GPIOA->ODR ^= 0x0002 ;
				LongCount = 0 ;
			}
		}
	}
}
