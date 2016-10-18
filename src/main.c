#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define ONEWIRE_USART3 // Должно быть объявлено ДО include "OneWire.h", чтобы был добавлен обработчик соответствующего прерывания
#include "OneWire.h"

/* STM32 в 72 MHz. */
static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOB, GPIOC, and AFIO clocks. */
    //rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable clocks for USARTs. */
    //rcc_periph_clock_enable(RCC_USART2); //включить, если используется отладка
#ifdef ONEWIRE_USART3
    rcc_periph_clock_enable(RCC_USART3);
#endif
}


static void gpio_setup(void) {
    //gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
    //              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX | GPIO_USART2_RX);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX | GPIO_USART3_RX);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

    /* Преконфигурация LED. */
    gpio_clear(GPIOC, GPIO13);
}

OneWire ow;

int main(void) {

    clock_setup();
    gpio_setup();

    ow.usart = USART3;
    owInit(&ow);

    //usart_setup(USART2, 115200, 8, USART_STOPBITS_1, USART_MODE_TX_RX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE); //отладочный USART

    uint32_t step=0, i;
    /* Blink the LEDs (PC13 and PB4) on the board. */
    while (1) {
        //printf(" attempt %lu, 1wire init said: %x\n", step, );
        owReset(&ow);
        owSearchCmd(&ow);
        int k = 10;
        while (k > 0) {
            gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
            uint32_t p = 800000;
            for (i = 0; i < p; i++)    /* Wait a bit. */
                    __asm__("nop");
            k--;
        }
        step++;
    }

    return 0;
}
