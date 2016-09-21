#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "OneWire.h"

/* Set STM32 to 72 MHz. */
static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOB, GPIOC, and AFIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    //rcc_periph_clock_enable(RCC_AFIO);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART2);
}

static void gpio_setup(void) {
    /// TX и RX устанавливаются отдельно, потому что работать должны в разных режимах
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX );

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART2_RX );

    /* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    //AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

    /* Preconfigure the LED. */
    gpio_clear(GPIOC, GPIO13);    /* Switch on LED. */
}

int main(void) {
    clock_setup();
    gpio_setup();

    int i;
    /* Blink the LEDs (PC13 and PB4) on the board. */
    while (1) {
        OneWireSend(USART2, 0xF0, 0x00, 0x00, 1);
        int k = 10;
        while (k > 0) {
            gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
            uint32_t p = 800000;
            for (i = 0; i < p; i++)    /* Wait a bit. */
                    __asm__("nop");
            k--;
        }
    }

    return 0;
}
