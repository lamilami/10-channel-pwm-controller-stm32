// STM32 4-channel Servo Demo for 24 MHz VL Discovery - sourcer32@gmail.com
 
#include "stm32F10x.h"
#include "delay.h"

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void TIM3_Configuration(void);
 
 
volatile float current_angle[10] = {0,0,0,0,0,0,0,0,0,0}; // +/- 90 degrees, use floats for fractional
volatile float delta_angle[10]={0,0,0,0,0,0,0,0,0};
volatile float init_angle[10]= {50,-2,0,10,-15,0,10,-60,45,0};

u32 left_step[10]={0,0,0,0,0,0,0,0,0};

 
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

	if(left_step[1]){
		current_angle[1]+= delta_angle[1];
		--left_step[1];
	}
	if(left_step[3]){
		current_angle[3]+= delta_angle[3];
		--left_step[3];
	}
	if(left_step[5]){
		current_angle[5]+= delta_angle[5];
		--left_step[5];
	}
	if(left_step[7]){
		current_angle[7]+= delta_angle[7];
		--left_step[7];
	}
    // minimum high of 600 us for -90 degrees, with +90 degrees at 2400 us, 10 us per degree
    //  timer timebase set to us units to simplify the configuration/math
    TIM3->CCR1 = 600 + ((current_angle[1] + 90) * 10); // where angle is an int -90 to +90 degrees, PC.6
    TIM3->CCR2 = 600 + ((current_angle[3] + 90) * 10); // where angle is an int -90 to +90 degrees, PC.7
    TIM3->CCR3 = 600 + ((current_angle[5] + 90) * 10); // where angle is an int -90 to +90 degrees, PC.8
    TIM3->CCR4 = 600 + ((current_angle[7] + 90) * 10); // where angle is an int -90 to +90 degrees, PC.9
  }
}							 




void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	if(left_step[0]){
		current_angle[0]+= delta_angle[0];
		--left_step[0];
	}
	if(left_step[2]){
		current_angle[2]+= delta_angle[2];
		--left_step[2];
	}
	if(left_step[4]){
		current_angle[4]+= delta_angle[4];
		--left_step[4];
	}
	if(left_step[6]){
		current_angle[6]+= delta_angle[6];
		--left_step[6];
	}
 
    // minimum high of 600 us for -90 degrees, with +90 degrees at 2400 us, 10 us per degree
    //  timer timebase set to us units to simplify the configuration/math
    TIM4->CCR1 = 600 + ((current_angle[0] + 90) * 10); // where angle is an int -90 to +90 degrees, PB.6
    TIM4->CCR2 = 600 + ((current_angle[2] + 90) * 10); // where angle is an int -90 to +90 degrees, PB.7
    TIM4->CCR3 = 600 + ((current_angle[4] + 90) * 10); // where angle is an int -90 to +90 degrees, PB.8
    TIM4->CCR4 = 600 + ((current_angle[6] + 90) * 10); // where angle is an int -90 to +90 degrees, PB.8
  }
}

void TIM5_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	if(left_step[8]){
		current_angle[8]+= delta_angle[8];
		--left_step[8];
	}
	if(left_step[9]){
		current_angle[9]+= delta_angle[9];
		--left_step[9];
	}
 
    // minimum high of 600 us for -90 degrees, with +90 degrees at 2400 us, 10 us per degree
    //  timer timebase set to us units to simplify the configuration/math
    TIM5->CCR1 = 600 + ((current_angle[8] + 90) * 10); // where angle is an int -90 to +90 degrees, PB.6
    TIM5->CCR2 = 600 + ((current_angle[9] + 90) * 10); // where angle is an int -90 to +90 degrees, PB.7
  }
}

void delay_quartersecond(u16 n)
{
	u16 i;
	for(i=0;i<n;i++)
		delay_ms(250);
}

void smooth(int channel, volatile float target, u32 lefttime)
{
		delta_angle[channel]=(target+init_angle[channel]-current_angle[channel])/(lefttime/20);
		left_step[channel]=	(lefttime/20);	
}
//
void rough(int channel, volatile float target)
{
	left_step[channel]=0;
	current_angle[channel]=target+init_angle[channel];
}

void test()
{
	current_angle[0]=10;
	current_angle[1]=-10;
	current_angle[2]=20;
	current_angle[3]=-20;
	current_angle[4]=30;
	current_angle[5]=-30;
	current_angle[6]=40;
	current_angle[7]=-40;
	current_angle[8]=50;
	current_angle[9]=-50;
}


 
void standsteady()
{
	rough(0,0);
	rough(1,0);
	rough(2,0);
	rough(3,0);
	rough(4,-5);
	rough(5,5);
	rough(6,5);
	rough(7,-5);
	rough(9,0);
	rough(8,0);

}

void forward()
{
	//while(1)
	{
		rough(7,20);
		rough(6,-20);
		smooth(0,30,500);

		delay_quartersecond(1);
		smooth(6, -70,500);
		smooth(4, -40,500);

		delay_quartersecond(1);

		smooth(7,28,500);
		smooth(5,-30,500);


		smooth(0,0,500);

		smooth(9,-20,500);

		delay_quartersecond(2);


		delay_quartersecond(5);
		//onstep
		smooth(1,-18,500);
		smooth(0,-5,500);
		//Gravity Point Swap

		delay_quartersecond(2);

		smooth(4,-30,1000);
		smooth(6,-20,1000);


		delay_quartersecond(2);
		smooth(1,-15,500);
		smooth(4,0,1000);
		smooth(7,70,1000);
		smooth(8,20,500);
		smooth(5,30,1000);
		delay_quartersecond(2);
		smooth(1,0,500);
		smooth(4,20,1000);
		smooth(6,-10,1000);
		smooth(3,10,1000);

		delay_quartersecond(4);
		smooth(1,0,1000);
		smooth(0,10,1000);
		

		delay_quartersecond(3);



		//smooth(4,-20,500);
		//smooth(6,-10,500)



		//Gravity Point Centered

		//smooth(7,70,500);
		//smooth(5,40,500);

		//smooth(4,10,500);
		//smooth(6,-12,500);




	//	smooth(2,20,500);


		/*delay_quartersecond(5);
		smooth(1,15,500);
		smooth(0,20,500);
	
		delay_quartersecond(2);

		rough(0,0);
	
	
		smooth(6,-20,1000);
		smooth(4,-30,1000);
		delay_quartersecond(2);

		smooth(8,25,500);

		delay_quartersecond(2);

		smooth(1,-5,1000);
		smooth(0,-5,1000);

		smooth(8,10,1000);
		smooth(2,-10,1000);
		smooth(3,10,1000);
		smooth(5,20,1000);
		smooth(3,25,1000);

		delay_quartersecond(4);

		smooth(0,-18,1000);
		smooth(1,-10,1000);

		delay_quartersecond(4);

		smooth(7,4
		0,1000);

		smooth(3,-10,1000);
		smooth(2,-15,1000);
		smooth(4,-25,1000);
		smooth(9,-20,1000);


		delay_quartersecond(4);

		smooth(0,0,1000);
		smooth(1,0,1000);

		smooth(7,20,1000);

		delay_quartersecond(4);

		

		
		//smooth(9,10,800);
		//smooth(8,10,800);

		//smooth(5,10,1000);
		//smooth(7,-10,1000);
	
	
		//delay_quartersecond(6);
	
		//smooth(1,0,250);
		//smooth(0,0,250);

		//delay_quartersecond(1);
		//smooth(1,-8,250);
		//smooth(0,-10,250);
		//delay_quartersecond(1);

		//smooth(4,-30,1000);
		//smooth(6,30,1000);
		//smooth(5,-30,1000);
		//smooth(7,-30,1000);

	
		//smooth(9,10,800);
	
		//delay_quartersecond(4);
		//smooth(5,25,300);
		//smooth(7,25,300);
	
		//delay_halfsecond(1);
	
	
	   	//smooth(5,10,800);
		//smooth(7,10,800);
		*/
	}

}



int main(void)
{
  delay_init(72);
  RCC_Configuration();
  GPIO_Configuration();
  NVIC_Configuration();
  TIM3_Configuration();
  standsteady();
  delay_quartersecond(8);  
  forward();

  while(1)
  {
		/*if(p==3000000)
		{
			if(sign)
			{
				servo_angle[0]=servo_angle[1]=servo_angle[2]=servo_angle[3]=-60;
				sign=0;
			}else
			{
				servo_angle[0]=servo_angle[1]=servo_angle[2]=servo_angle[3]=0;
				sign=1;
			}
			p=0;
		}
		p++;
		*/
  }
}
 
/**************************************************************************************/
 
void RCC_Configuration(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC |RCC_APB2Periph_GPIOD| RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
}
 
/**************************************************************************************/
 
void NVIC_Configuration(void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
 
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
 
   NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	// PC.06 TIM3_CH1
	// PC.07 TIM3_CH2
	// PC.08 TIM3_CH3
	// PC.09 TIM3_CH4

 	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);
	
	//	PB.06 TIM4_CH1
	//	PB.07 TIM4_CH1
	//	PB.08 TIM4_CH1
	//	PB.09 TIM4_CH1



	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8 |GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 |GPIO_Pin_1;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


}
 
/**************************************************************************************/
 
void TIM3_Configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;  // 24 MHz / 24 = 1 MHz
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20 ms)
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);


  // Set up 4 channel servo
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 600 + 900; // 1500 us - Servo Top Centre
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);    // Channel 2 configuration = PC.06 TIM3_CH1
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);    // Channel 4 configuration = PC.07 TIM3_CH2
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);    // Channel 6 configuration = PC.08 TIM3_CH3
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);    // Channel 8 configuration = PC.09 TIM3_CH4

	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);    // Channel 1 configuration = PD.12 TIM4_CH1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);    // Channel 3 configuration = PD.13 TIM4_CH2
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);    // Channel 5 configuration = PD.14 TIM4_CH3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);	// Channel 7 configuration = PD.15 TIM4_CH4

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);    // Channel 1 configuration = PD.12 TIM5_CH1
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);    // Channel 3 configuration = PD.13 TIM5_CH2

    // turning on TIM3 and PWM outputs
 
	TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
  	// TIM IT enable
  	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // turning on TIM4 and PWM outputs	
	TIM_Cmd(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
  	// TIM IT enable
  	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		
	TIM_Cmd(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM5, ENABLE);
  	// TIM IT enable
  	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}
