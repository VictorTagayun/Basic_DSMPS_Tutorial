## Basic tutorial and Setup of STM32 MCU for Digital SMPS application

### What will this tutorial will do and not do

Will DO
	Teach how to setup the HRTIMer, its freq, PLL etc.
	Use the GPIO for timing and debugging
	How to setup period and duty cycle control
	Use RC circuit as a simulated SMPS circuits
	Observe "exactly" where in time the ADC is acquiring data

Will not do
	Teach about the basic of SMPS, it theory, operations, formulas, etc
	Teach about feedback control theory
	Use actual Power devices to convert power
	Give overview of the evaluation board
	
We will use NUCLEO-F334R8 as the development board where the external Freq used is 8MHz, while the IC has a maximum System Freq of 72MHz

### Setup RCC

With 8Mhz input clock and Max 72Mhz of system freq we set the ff:

PLL = 8 (default and cannot change)
PLLMUL = 9

	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  

### Setup GPIOC9 and check and compare SPEED

Check which code will yield the fastest GPIO speed, pls refer to [](https://github.com/VictorTagayun/STM32F334_HAL_LL_REG_GPIO)

Toggle

	  // HAL = 878 KHz
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	  // HAL = 922 kHz
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

	  // LL = 1.18MHz
//	  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
//	  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);

	  // LL = 922 kHz
//	  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);

	  // LL  = 5.54Mhz
	  // HAL = 5.54Mhz
//	  GPIOA->BSRR = (1<<8);
//	  GPIOA->BSRR = (1<<24);

	  // HAL = 5.54Mhz
	  // LL  = 6.00Mhz
	  GPIOA->BSRR = (1<<8); // Set
	  GPIOA->BRR = (1<<8); // Reset

## Setup HRTIMer

Set PLL as source (with integrated multiplier of 2)

	PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
	
Set prescaler to multiplier of 32

	pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
	
At this moment HRTIMer freq is running at 4.608Ghz or 217pS, 72MHz x 2 x 32. We mutiplied to this high freq to have a resolution of 217pS.

The period register in HRTIMer is 16bit register, that means it can count to 2^16 = 65536, theoretically. That is because some bits/byte combination are lost, so just nearly 65,536.

To operate at 100kHz (10us) switching freq, we need 46,080 clock cycles. It is with in the 65536 max counter limit.

If we exeed that couter, we will use a prescaler multiplier of 16 intead.

That means the period we have now is 46,080 x 217pS. When the counter reach 46,080, it would return to 0. It also means we have a duty cycle resolution of 1/46,080. That's 0.0217%!

	pTimeBaseCfg.Period = 46080;

We will use Comparator1 and Comparator3, Comparator1 for the max duty cycle while Comparator3 for setting the duty cycle. 

We set Comparator1 to be less 1000 of the duty cycle, for safety.

	pCompareCfg.CompareValue = 46080 - 1000;
	
We set Comparator3 at the meantime 50%.

	pCompareCfg.CompareValue = 46080/2;
	
For the GPIO output setting, we set HIGH the output when the HRTIMer readh the period and set LOW when it reach Comparator1 "OR" Comparator3.

	pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
	pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1|HRTIM_OUTPUTRESET_TIMCMP3;
	
In the main.c we start the HRTIMer counter and enagle its GPIO output.


	