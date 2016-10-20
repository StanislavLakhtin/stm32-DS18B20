#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// Должно быть объявлено ДО include "OneWire.h", чтобы был добавлен обработчик соответствующего прерывания
//#define ONEWIRE_UART5
//#define ONEWIRE_UART4
#define ONEWIRE_USART3
//#define ONEWIRE_USART2
//#define ONEWIRE_USART1

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

    DS18B20_Scratchpad data; //buffer for reading scratchpad's
    bool readWrite = true;
    uint32_t pDelay, i;

    while (1) {
        if (owResetCmd(&ow) != ONEWIRE_NOBODY) {    // is anybody on the bus?
            owSearchCmd(&ow);                       // take them romId's
            for (i = 0; i < MAXDEVICES_ON_THE_BUS; i++) {
                if (ow.ids[i].family == DS18B20) {
                    if (readWrite) {
                        //owWriteDS18B20ScratchpadCmd(&ow, &ow.ids[0], 0xff, 0xff, 0x7f); //TH = 0x20 TL=0x40 CONF (R1R2=00) 9bit precise
                        owConvertTemperatureCmd(&ow, &ow.ids[i]);
                        pDelay = 2500000;
                    } else {
                        owReadScratchpadCmd(&ow, &ow.ids[i], (uint8_t *) &data);
                        pDelay = 1000000;
                    }
                }
            }
        } else {
            pDelay = 8000000;
        }
        //do something while sensor work
        int k = 10;
        while (k > 0) {
            gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
            for (i = 0; i < pDelay; i++)    /* Wait a bit. */
                    __asm__("nop");
            k--;
        }
        readWrite = !readWrite;
    }

    return 0;
}
