///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//設置STM32輸入/輸出捕獲模式暫存器

void timer_setup(void) {
  Timer2.attachCompare1Interrupt(handler_channel_1);                   //這是四個通道分別對應的中斷 
  Timer2.attachCompare2Interrupt(handler_channel_2);
  Timer2.attachCompare3Interrupt(handler_channel_3);
  Timer2.attachCompare4Interrupt(handler_channel_4);
  TIMER2_BASE->CR1 = TIMER_CR1_CEN;                       //細部的暫存器設置 包括設置模式/最大計數值/分頻器 上升沿捕抓與下降沿補抓
  TIMER2_BASE->CR2 = 0;
  TIMER2_BASE->SMCR = 0;
  TIMER2_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE | TIMER_DIER_CC3IE | TIMER_DIER_CC4IE;
  TIMER2_BASE->EGR = 0;
  TIMER2_BASE->CCMR1 = 0b100000001; 
  TIMER2_BASE->CCMR2 = 0b100000001; 
  TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER2_BASE->PSC = 71;
  TIMER2_BASE->ARR = 0xFFFF;
  TIMER2_BASE->DCR = 0;

  Timer3.attachCompare1Interrupt(handler_channel_5);                     //這兩個通道可以隨意使用 
  Timer3.attachCompare2Interrupt(handler_channel_6);
  TIMER3_BASE->CR1 = TIMER_CR1_CEN;
  TIMER3_BASE->CR2 = 0;
  TIMER3_BASE->SMCR = 0;
  TIMER3_BASE->DIER = TIMER_DIER_CC1IE | TIMER_DIER_CC2IE;                                            
  TIMER3_BASE->EGR = 0;
  TIMER3_BASE->CCMR1 = 0b100000001;                                       //細部的暫存器設置 包括設置模式/最大計數值/分頻器 上升沿捕抓與下降沿補抓
  TIMER3_BASE->CCMR2 = 0;
  TIMER3_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E;
  TIMER3_BASE->PSC = 71;
  TIMER3_BASE->ARR = 0xFFFF;
  TIMER3_BASE->DCR = 0;
  //timer4
  TIMER4_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
  TIMER4_BASE->CR2 = 0;
  TIMER4_BASE->SMCR = 0;
  TIMER4_BASE->DIER = 0;
  TIMER4_BASE->EGR = 0;
  TIMER4_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE |(0b110 << 12) | TIMER_CCMR1_OC2PE;
  TIMER4_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE |(0b110 << 12) | TIMER_CCMR2_OC4PE;
  TIMER4_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
  TIMER4_BASE->PSC = 71;
  TIMER4_BASE->ARR = 5000;
  TIMER4_BASE->DCR = 0;
  TIMER4_BASE->CCR1 = 1000;

  TIMER4_BASE->CCR1 = 1000;      
  TIMER4_BASE->CCR2 = 1000;
  TIMER4_BASE->CCR3 = 1000;
  TIMER4_BASE->CCR4 = 1000;
  pinMode(PB6, PWM);        //輸出給ESC 故為PWM輸出
  pinMode(PB7, PWM);
  pinMode(PB8, PWM);
  pinMode(PB9, PWM);
}

