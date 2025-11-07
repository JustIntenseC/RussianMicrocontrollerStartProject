#include "K1921VK035.h"
void Led_Pin_Toggle(void);
static void Led_Pin_Init(void);
static void Button_Init(void);
void Led_Pin_ON(void);
void Led_Pin_OFF(void);
int main(void){
    
    SystemCoreClockUpdate();
    Led_Pin_Init();
    Button_Init();
    SysTick_Config(10000000);



    while(1){

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
    GPIOA->INTTYPESET_bit.PIN7 = 1; //front
    GPIOA->INTPOLSET_bit.PIN7 = 0;
    GPIOA->INTENSET_bit.PIN7 = 1;
    NVIC_EnableIRQ(GPIOA_IRQn);
}
void GPIOA_IRQHandler(void){
     Led_Pin_Toggle();

}
void Led_Pin_ON(void){
    GPIOA->DATAOUTSET_bit.PIN8 = 1;
}
void Led_Pin_OFF(void){
    GPIOA->DATAOUTSET_bit.PIN8 = 0;
}
