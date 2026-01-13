#include "stm32l4xx.h"
#include <stdio.h>
#include "uart.h"

#define GPIOAEN  (1U << 0)
#define TIM2EN   (1U << 0)
#define SYSCNFG  (1U << 0)

volatile int pwm;
volatile int pulsecount =0;
int setpoint=300;
int error=0;
int speed=0;
int integ=0;
int deriv=0;
int preverr=0;

int Kp=10;
int Ki=10;
int Kd=10;

void delay(volatile uint32_t t)
{
    while (t--);
}

void GPIO_EN(void)
{
    RCC->AHB2ENR |= GPIOAEN;


    GPIOA->MODER &= ~((3U << 2) | (3U << 8));  //PA1 AND PA4 output mode
    GPIOA->MODER |=  ((1U << 2) | (1U << 8));


    GPIOA->MODER &= ~(3U << 0);  //PAO
    GPIOA->MODER |=  (1U << 1);

    GPIOA->AFR[0] |=  (1U << 0);    //AFmode
    GPIOA->AFR[0] &= ~(1U << 1);
    GPIOA->AFR[0] &= ~(1U << 2);
    GPIOA->AFR[0] &= ~(1U << 3);


    GPIOA->MODER &= ~((3U << 12) | (3U << 14)); //PA6 PA7


    GPIOA->PUPDR &= ~((3U << 12) | (3U << 14)); //Input mode
    GPIOA->PUPDR |=  ((1U << 12) | (1U << 14));
}

void Encoder_EXTI_INPUT(void)
{
    RCC->APB2ENR |= SYSCNFG;
                                 //PA6 for interrupt

    SYSCFG->EXTICR[1] &= ~(15U << 8);

    EXTI->IMR1  |= (1U << 6);
    EXTI->FTSR1 |= (1U << 6);

    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << 6))
    {
        EXTI->PR1 |= (1U << 6);

        int A = (GPIOA->IDR & (1U << 6)) ? 1 : 0;
        int B = (GPIOA->IDR & (1U << 7)) ? 1 : 0;

        if (A == B)
            pulsecount++;
        else
            pulsecount--;

    }
}

void TIM_EN(void)
{
    RCC->APB1ENR1 |= TIM2EN;

    TIM2->PSC  = 0;
    TIM2->ARR  = 20000;
    TIM2->CNT  = 0;

    TIM2->CCMR1 &= ~(7U << 4);
    TIM2->CCMR1 |=  (1U << 5) | (1U << 6);  // PWM mode
    TIM2->CCMR1 |=  (1U << 3);

    TIM2->CCR1 = 10000;
    TIM2->CCER |= (1U << 0);

    TIM2->CR1 |= (1U << 7);  // ARPE
    TIM2->CR1 |= (1U << 0);  // Counter enable
}

void dir(void)
{
    GPIOA->ODR |=  (1U << 1);   //Clockwise
    GPIOA->ODR &= ~(1U << 4);
}


void PID(void){
	speed=pulsecount;


	error=setpoint-speed;
	  integ += error;
	   if (integ > 1000) integ = 1000;
	    if (integ < -1000) integ = -1000;

	  deriv=error-preverr;

	    int pid= Kp * error + Ki * integ+ Kd* deriv;
	    preverr = error;

	    pwm =pid;
	    if (pwm < 0) pwm = 0;
	    if (pwm > 20000) pwm = 20000;


	    TIM2->CCR1 = pwm;

}

int main(void)
{
    GPIO_EN();
    TIM_EN();
    Encoder_EXTI_INPUT();
    uart_tx_init();
    dir();

    while (1)
    {
    	PID();
        printf("Pulse=%d Speed=%d Error=%d PWM=%d\n\r ",  pulsecount, speed, error,pwm);


        delay(50000);
    }
}

