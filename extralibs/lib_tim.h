void TIM_init(TIM_TypeDef * TIMx, u16 prescaler, u16 period) {
	// Enable TIMER clock
	if (TIMx == TIM1) {
		RCC->APB2PCENR |= RCC_APB2Periph_TIM1;
	}
	#ifdef TIM2
	else if (TIMx == TIM2) {
		RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	}
	#endif

	TIMx->CTLR1 = (TIMx->CTLR1 & ~TIM_DIR) | TIM_CounterMode_Up;
	TIMx->PSC = prescaler - 1;
	TIMx->ATRLR = period - 1;
	TIMx->SWEVGR = TIM_UG;
}

void TIM_CompareCapture_config(
	TIM_TypeDef *TIMx, u16 timer_channel,
	u16 selection, u16 mode, u16 prescaler, u16 filter, u16 value
) {
	switch (timer_channel) {
		case TIM_Channel_1:
			TIMx->CHCTLR1 &= ~(TIM_CC1S | TIM_OC1M | TIM_IC1PSC | TIM_IC1F);
			TIMx->CHCTLR1 |= (selection) | (mode) | (prescaler) | (filter << 4);
			TIMx->CH1CVR = value;
			break;
		case TIM_Channel_2:
			TIMx->CHCTLR1 &= ~(TIM_CC2S | TIM_OC2M | TIM_IC2PSC | TIM_IC2F);
			TIMx->CHCTLR1 |= (selection << 8) | (mode << 8) | (prescaler << 8) | (filter << 12);
			TIMx->CH2CVR = value;
			break;
		case TIM_Channel_3:
			TIMx->CHCTLR2 &= ~(TIM_CC3S | TIM_OC3M | TIM_IC3PSC | TIM_IC3F);
			TIMx->CHCTLR2 |= (selection) | (mode) | (prescaler) | (filter << 4);
			TIMx->CH3CVR = value;
			break;
		case TIM_Channel_4:
			TIMx->CHCTLR2 &= ~(TIM_CC4S | TIM_OC4M | TIM_IC4PSC | TIM_IC4F);
			TIMx->CHCTLR2 |= (selection << 8) | (mode << 8) | (prescaler << 8) | (filter << 12);
			TIMx->CH4CVR = value;
			break;
	}
}

// Output Compare Polarity
void TIM_OCPolarity_config(TIM_TypeDef *TIMx, u16 name, u16 polarity, u16 enabled) {
	// polarity is preshifted by 1
	switch (name) {
		case TIM_CC1P:
			TIMx->CCER &= ~(TIM_CC1P | TIM_CC1E);
			TIMx->CCER |= (polarity << 0) | (enabled << 0);
			return;
		case TIM_CC2P:
			TIMx->CCER &= ~(TIM_CC2P | TIM_CC2E);
			TIMx->CCER |= (polarity << 4) | (enabled << 4);
			return;
		case TIM_CC3P:
			TIMx->CCER &= ~(TIM_CC3P | TIM_CC3E);
			TIMx->CCER |= (polarity << 8) | (enabled << 8);
			return;
		case TIM_CC4P:
			TIMx->CCER &= ~(TIM_CC4P | TIM_CC4E);
			TIMx->CCER |= (polarity << 12) | (enabled << 12);
			return;
	}

	// for regular polarity low, expects TIM_OCPolarity_Low
	// for Complementary polarity low expects TIM_OCNPolarity_Low
	if (polarity > 0) polarity = TIM_OCNPolarity_Low;

	//# Only advanced timers (ADTM) 1, 8, 9 and 10 have Complementary output polarity
	if((TIMx != TIM1) && (TIMx != TIM8) && (TIMx != TIM9) && (TIMx != TIM10)) return;

	switch (name) {
		case TIM_CC1NP:
			TIMx->CCER &= ~(TIM_CC1NP | TIM_CC1NE);
			TIMx->CCER |= (polarity << 0) | (enabled << 0);
			break;
		case TIM_CC2NP:
			TIMx->CCER &= ~(TIM_CC2NP | TIM_CC2NE);
			TIMx->CCER |= (polarity << 4) | (enabled << 4);
			break;
		case TIM_CC3NP:
			TIMx->CCER &= ~(TIM_CC3NP | TIM_CC3NE);
			TIMx->CCER |= (polarity << 8) | (enabled << 8);
			break;
	}
}

// Output Compare Idle
void TIM_OCIdle_config(
	TIM_TypeDef *TIMx, u16 timer_channel,
	u16 name, u16 state, u16 nState
) {
	switch (timer_channel) {
		case TIM_Channel_1:
			TIMx->CTLR2 &= ~(TIM_OIS1 | TIM_OIS1N);
			TIMx->CTLR2 |= (state << 0) | (nState << 0);
			return;
		case TIM_Channel_2:
			TIMx->CTLR2 &= ~(TIM_OIS2 | TIM_OIS2N);
			TIMx->CTLR2 |= (state << 2) | (nState << 2);
			return;
		case TIM_Channel_3:
			TIMx->CTLR2 &= ~(TIM_OIS3 | TIM_OIS3N);
			TIMx->CTLR2 |= (state << 4) | (nState << 4);
			return;
		case TIM_Channel_4:
			// there is no TIM_OIS4N
			TIMx->CTLR2 &= ~(TIM_OIS4);
			TIMx->CTLR2 |= (state << 6);
			return;
	}
}
