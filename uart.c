#include <stdint.h>
#include "stm32l4xx.h"
#include "uart.h"



#define GPIOAEN  (1U<<0)
#define UART2EN (1U<<17)
#define SYS_FREQ  4000000
#define PCLK  SYS_FREQ

#define CR1_TE  (1U<<3)
#define CR1_UE  (1U<<0)
#define ISR_TXE (1U<<7)

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t Pclk ,uint32_t Bdr);
void uart_tx_init();
static uint16_t uart_comp_div(uint32_t Pclk,uint32_t Bdr);
static void uart_write(USART_TypeDef *USARTx,uint8_t ch);

int __io_putchar(int ch){
	 uart_write(USART2, ch);
	 return ch;
}

void uart_tx_init(void){
	RCC->AHB2ENR |=GPIOAEN;

	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |=(1U<<5);

	GPIOA->AFR[0] |=(1U<<8);
	GPIOA->AFR[0] |=(1U<<9);
	GPIOA->AFR[0] |=(1U<<10);
	GPIOA->AFR[0] &=~(1U<<11);

	RCC->APB1ENR1 |=UART2EN;
	uart_set_baudrate(USART2,PCLK ,115200);

	USART2->CR1 =CR1_TE;
	USART2->CR1 |=CR1_UE;

}


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t Pclk ,uint32_t Bdr){
	USARTx->BRR = uart_comp_div(Pclk,Bdr);
}

static uint16_t uart_comp_div(uint32_t Pclk,uint32_t Bdr){
	return ((Pclk+(Bdr/2))/Bdr);
}

static void uart_write(USART_TypeDef *USARTx,uint8_t ch){
	while(!(USARTx->ISR & ISR_TXE)){

	}
	USARTx->TDR = (ch & 0xFF);
}


