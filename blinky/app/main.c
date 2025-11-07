#include "K1921VK035.h"
#include <stdio.h>
#include <string.h>
typedef enum{
    STATUS_OFF,
    STATUS_ON
} LED_STATUS;

uint32_t status = 0;
void Led_Pin_Toggle(void);
static void Led_Pin_Init(void);
static void Button_Init(void);
LED_STATUS Led_Pin_ON(void);
LED_STATUS Led_Pin_OFF(void);
static void UART_Init(void);
void UART_SendByte(uint8_t *Byte);
void UART_SendMessage(uint8_t *msg, uint16_t lenght);
static void Timer_Init(void);
char msg[64] = "Hello from K1921VK035 microcontroller\r\n";

#define MAX_PACKET_SIZE 256
volatile uint8_t packet_buffer[MAX_PACKET_SIZE];
volatile uint16_t packet_index = 0;
volatile uint8_t packet_received = 0; 
volatile uint8_t Date_UART_Receive=0;
void Delay(void);
int main(void){
    
    SystemCoreClockUpdate();
    Led_Pin_Init();
    Button_Init();
    SysTick_Config(10000000);
    Led_Pin_ON();
    UART_Init();
    UART_SendMessage(msg, strlen(msg));
    while(1){


        if(packet_received){
            UART_SendMessage(packet_buffer, strlen(packet_buffer));
            packet_received = 0;
            memset(packet_buffer, 0, sizeof(msg));
        }
       
    
    }
    return 0;
}

void SysTick_Handler(void){
    // Led_Pin_Toggle();
}

void Led_Pin_Toggle(void){
    GPIOA->DATAOUTTGL_bit.PIN8  = 1;
}
static void Led_Pin_Init(void){

    RCU->HCLKCFG_bit.GPIOAEN = 1;
    RCU->HRSTCFG_bit.GPIOAEN = 1;
    GPIOA->DENSET_bit.PIN8 = 1;
    GPIOA->OUTENSET_bit.PIN8 = 1;

}
static void Button_Init(void){

    RCU->HCLKCFG_bit.GPIOAEN = 1;
    RCU->HRSTCFG_bit.GPIOAEN = 1;
    GPIOA->DENSET_bit.PIN7 = 1;
    GPIOA->INTTYPESET_bit.PIN7 = 0; 
    GPIOA->INTPOLSET_bit.PIN7 = 0;
    GPIOA->INTENSET_bit.PIN7 = 1;
    NVIC_EnableIRQ(GPIOA_IRQn);

}
void GPIOA_IRQHandler(void){
    GPIOA->INTSTATUS_bit.PIN7 = 1;
    Led_Pin_Toggle();
    status = (GPIOA->DATA & (1<<7)) >> 7;
        if(status == STATUS_ON){
            Led_Pin_OFF();
        }

}
LED_STATUS Led_Pin_ON(void){

    GPIOA->DATAOUTSET_bit.PIN8 = 1;
    return STATUS_ON;

}
LED_STATUS Led_Pin_OFF(void){

    GPIOA->DATAOUTCLR_bit.PIN8 = 1;
    return STATUS_OFF;
}

static void UART_Init(void){

    RCU->HCLKCFG_bit.GPIOBEN = 1;
    RCU->HRSTCFG_bit.GPIOBEN = 1;

    GPIOB->ALTFUNCSET_bit.PIN10 = 1; // TRANSMIT
    GPIOB->DENSET_bit.PIN10 = 1;

    GPIOB->ALTFUNCSET_bit.PIN11 = 1; // RECEIVE
    GPIOB->DENSET_bit.PIN11 = 1;
  
    
    // RCU->UARTCFG[1].UARTCFG |= (0<<16)|(1<<8)|(1<<4)|(1<<0);
    RCU->UARTCFG[0].UARTCFG_bit.DIVEN = 0;
    RCU->UARTCFG[0].UARTCFG_bit.CLKSEL = RCU_UARTCFG_UARTCFG_CLKSEL_PLLCLK;
    RCU->UARTCFG[0].UARTCFG_bit.RSTDIS = 1;
    RCU->UARTCFG[0].UARTCFG_bit.CLKEN = 1;
    
    uint32_t cpuClock = 100000000U;
    uint32_t realSpeed = 115200U;
    uint32_t integerDivider = cpuClock / (16 * realSpeed);
    uint32_t fractionalDivider = (uint32_t)((cpuClock / (16.0f *realSpeed) - integerDivider) * 64.0f + 0.5);
    UART0->IBRD = integerDivider;
    UART0->FBRD = fractionalDivider;

    UART0->LCRH_bit.WLEN = UART_LCRH_WLEN_8bit;
    UART0->LCRH_bit.STP2 = 0U;

    UART0->CR_bit.RXE = 1;
    UART0->CR_bit.TXE = 1;

    UART0->IMSC_bit.RXIM = 1;

    NVIC_SetPriority(UART0_RX_IRQn, 0);
    NVIC_EnableIRQ(UART0_RX_IRQn);

    UART0->CR_bit.UARTEN = 1;

   
}

void UART0_RX_IRQHandler(void) {
    Led_Pin_Toggle();

    if (packet_index < MAX_PACKET_SIZE){
        uint8_t byte = UART0->DR & 0xFF;
         packet_buffer[packet_index] = byte;
        if(byte == '\n'){
            packet_index = 0;
            packet_received = 1;
        }
        else{
            packet_index ++;
        }
    }
    UART0->ICR_bit.RXIC = 1;
  
}

void UART_SendByte(uint8_t *Send_Byte) {
    uint32_t timeout = 1000000;
    while ((UART0->FR_bit.BUSY == 1) && timeout--) {};
    if (timeout == 0) return; 
    UART0->DR = *Send_Byte;
}

void UART_SendMessage(uint8_t *msg, uint16_t length) {
    for (int i = 0; i < length; i++) {
        UART_SendByte(&(msg[i]));
    }
}


static void Timer_Init(void){
//    
}

void Delay(void){
    for (volatile int i = 0; i < 10000000; i++);  

}
