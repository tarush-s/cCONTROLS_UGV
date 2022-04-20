#include "stm32f10x.h"
#include "stdlib.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
void MotorCode(int x, int y, float gear);
void GPIO_Initialize(void);
void Timer_Initialize(void);
void UART_Initilaize(void);
uint8_t getuval(void); 
volatile static int count=0;
int flag=0;
void SysTick_Handler(void)
{
	count++;
}

void dms(int ms)
{
	count=0;
	while(count<ms);
}
void GPIO_Initialize()
{

	// CONFIGURE PORT C PIN 13 -> ONBOARD LED
	RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN;
	GPIOC -> CRH |= GPIO_CRH_MODE13;
	GPIOC -> CRH &= ~(GPIO_CRH_CNF13);

	
	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Enable Clock for Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable Clock for Port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function

	//Setup PA4:
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB6:
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

	//PA10 Setup: (UART Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA->CRH |= GPIO_CRH_CNF10;   //Input with pull-up/pull-down (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);

	//PB4 Setup:
	//Disable SWD & JTAG:
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);
	GPIOB->CRL |= (GPIO_CRL_MODE4);   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	GPIOB->BRR = 1 << (4);   //Mandatory turn off
}

void Timer_Initialize()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 and 2 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2

	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1;   //freq/1 = 8 Mhz
	TIM4->ARR = 8000;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void UART_Initilaize()
{
	/*
    	//PA9(Tx) PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 8Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 4<<4 | 5<<0;   //Gives 115200 Baud Rate(approx.) Register Value = (8MHz)/(16 * Reqd. Baud Rate) = 4.5
	//              Rx Enable      Tx Enable	  UART Enable
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
	*/
	
	
	// CONFIGURE UART | TX -> A9 | RX -> A10
	RCC ->APB2ENR |= RCC_APB2ENR_USART1EN;

	// RX - A10 | FLOATING INPUT
	GPIOA -> CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10_1);
	GPIOA -> CRH |= GPIO_CRH_CNF10_0;
	
	// TX - A9 | ALTERNATE OUTPUT PUSH-PULL
	GPIOA -> CRH &= ~GPIO_CRH_CNF9_0;
	GPIOA -> CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1;
	
	// BAUD RATE
	USART1 -> BRR |= 0x271; // 115200
	
	// CONTROL REGISTER | USART ENABLE | TRANSMIT ENABLE | RECEIVE ENABLE
	USART1 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; 
	
}

void MotorCode(int x, int y, float gear)
{
// STOP
	if (abs(x) < 20 && abs(y) < 20)
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = 0;  //Left SPEED
	  TIM4->CCR2 = 0;   //Right SPEED
	}		
	
  
	// FORWARD MAX
	else if(abs(x) < 10 && y > 0)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = ((uint32_t) abs(y)*(gear*0.1)) ;  //Left SPEED
	  TIM4->CCR2 = ((uint32_t) abs(y)*(gear*0.1));   //Right SPEED
		
	}	
	
	// BACKWARD MAX
	else if(abs(x) < 10 && y < 0) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(y)*(gear*0.1));  //Left SPEED
	  TIM4->CCR2 = (uint32_t)  (abs(y)*(gear*0.1));   //Right SPEED
	}	
	
	// SPOT LEFT
	else if (x < 0 && abs(y) <= 10) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(x)*(gear*0.1));  //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(x)*(gear*0.1));   //Right SPEED
	}		
	
	// SPOT RIGHT
	else if (x > 0 && abs(y) <= 10)
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(x)*(gear*0.1));		//Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(x)*(gear*0.1));   //Right SPEED		
	}		
		
	// OCTET 1
	else if(x > 0 && y > 0 && x > y) 
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
	  TIM4->CCR1 = (uint32_t) (abs(x)*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Right SPEED
	}		

	// OCTET 2
	else if(x > 0 && y > 0 && x < y) 
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(y)*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Right SPEED
	}		

	// OCTET 3
	else if(x < 0 && y > 0 && abs(x) < y)
	{
	  GPIOA->BSRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(y)*(gear*0.1));   //Right SPEED  
	}

  // OCTET 4
	else if(x < 0 && y > 0 && abs(x) >= y) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(x)*(gear*0.1));   //Right SPEED
	}	

  // OCTET 5	
	else if(x < 0 && y < 0 && abs(x) > abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BSRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(x)*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Right SPEED
	}
	
	// OCTET 6
	else if(x < 0 && y < 0 && abs(x) < abs(y))
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(y)*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Right SPEED
	}
	
	// OCTET 7
	else if(x > 0 && y < 0 && abs(x) < abs(y)) 
	{
		GPIOA->BRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(y)*(gear*0.1));   //Right SPEED
	}

	// OCTET 8
	else if(x > 0 && y < 0 && abs(x) > abs(y)) 
	{
		GPIOA->BSRR |= 1 << 4;
		GPIOA->BRR |= 1 << 5;
		TIM4->CCR1 = (uint32_t) (abs(abs(x) - abs(y))*(gear*0.1));   //Left SPEED
	  TIM4->CCR2 = (uint32_t) (abs(x)*(gear*0.1));   //Right SPEED
	}
}
uint8_t getuval()   //Reads UART Values
{
	uint8_t data;
	count=0;
	if(flag==1)
	{	
		return 0;
	}
	while(count<1000 && !(USART1->SR & USART_SR_RXNE))
	{
	}
	if(count>=1000)
	{
		flag=1;
	}
	else
	{
		flag=0;
		data = USART1->DR;
	 return data;
	}
	
}
int main()
{
	GPIO_Initialize();
	Timer_Initialize();
  UART_Initilaize();
	SysTick_Config(SystemCoreClock/1000);
	int x = 0, y = 0;
  int trash = 0;
	float gear = 1.0;
	while (1)
  	{
			
		//Read LAN2UART Values
      flag=0;
		if(getuval() == 'm')
		{
			gear = (int) ((getuval() - '0') + 1);   //Get gear value
			if(getuval() == 's')
			{
				x = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //x value
			}
			if(getuval() == 'f')
			{
				y = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //y value
			}
			trash = getuval();   //This is actually Mast CAM values but we're ignoring it for now
		}
		else
		{
			trash = trash + 1 - 1;   //Random values
		}
		if(flag==1)
		{
			x=8000;
			y=8000;
		}
		x = x - 8000;
		y = y - 8000;

		if(abs(x) < 500)
			x = 0;
		if(abs(y) < 500)
			y = 0;

		MotorCode(x, y, gear);   //Run MotorCode

		}
}
