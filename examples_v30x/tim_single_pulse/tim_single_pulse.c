#include "ch32fun.h"
#include <stdio.h>
#include "lib_tim.h"

#define PWM_PIN PA9
#define TIM1CH1_INPUT_PIN PA8

int main() {
	SystemInit();
	funGpioInitAll(); // Enable GPIOs

	printf("\n~ ADC Group Injection Example ~\n");
	printf("Chip ID: %08lX\n", ESIG->UID0);
	printf("Chip Capacity: %d KB\n", ESIG->CAP);

	funPinMode(PWM_PIN, GPIO_CFGLR_OUT_10Mhz_AF_PP);
	funPinMode(TIM1CH1_INPUT_PIN, GPIO_CFGLR_IN_PUPD);
	funDigitalWrite(TIM1CH1_INPUT_PIN, 1);

	TIM_init(TIM1, 48000, 200);
	TIM_CompareCapture_init(
		TIM1, TIM_Channel_1,
		TIM_ICSelection_DirectTI, TIM_OCMode_PWM2,
		TIM_ICPSC_DIV1, 0x00, 100
	);

	TIM_OCPolarity_config(TIM1, TIM_CC1P, TIM_ICPolarity_Falling, 1);

	// one pulse mode
	TIM1->CTLR1 |= (TIM1->CTLR1 & ~TIM_OPM) | TIM_OPMode_Single;
	TIM1->SMCFGR &= ~(TIM_SMS | TIM_TS);
	TIM1->SMCFGR = TIM_SlaveMode_Trigger | TIM_TS_TI1FP1;

	while(1) {
		printf("TIM1: %d\n", TIM1->CNT);
		Delay_Ms(1000);
	}
}
