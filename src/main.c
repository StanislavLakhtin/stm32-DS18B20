#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>


/* Set STM32 to 72 MHz. */
static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOB, GPIOC, and AFIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART2);
}

void usart_enable_halfduplex(uint32_t usart) {
    USART_CR3(usart) |= USART_CR3_HDSEL;
}

static void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity,
                        uint32_t flowcontrol) {
    usart_disable(usart);
    /* Setup parameters. */
    usart_set_baudrate(usart, baud);
    usart_set_databits(usart, bits);
    usart_set_stopbits(usart, stopbits);
    usart_set_mode(usart, mode);
    usart_set_parity(usart, parity);
    usart_set_flow_control(usart, flowcontrol);

    /* Finally enable the USART. */
    usart_enable_halfduplex(usart);
    usart_enable(usart);
}

static void gpio_setup(void) {
    /* Setup GPIO pin GPIO_USART2_TX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

    /* Set GPIO8 (in GPIO port A) to 'output push-pull'. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

    /* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

    /* Preconfigure the LED. */
    gpio_clear(GPIOC, GPIO13);    /* Switch on LED. */
}

int oneWireReset(uint32_t usart) {
    int oneWireDevices = 0;
    usart_setup(usart, 9600, 8, USART_STOPBITS_1, USART_MODE_TX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    usart_send_blocking(usart, 0xf0);
    while (!usart_get_flag(usart, USART_SR_TC));
    oneWireDevices = usart_recv(usart);

    usart_setup(usart, 115200, 8, USART_STOPBITS_1, USART_MODE_TX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    return oneWireDevices;
}

int main(void) {
    clock_setup();
    gpio_setup();
    int owDev = oneWireReset(USART2);

    int i;
    /* Blink the LEDs (PC13 and PB4) on the board. */
    while (1) {
        int k = 10 + owDev;
        while (k > 0) {
            //usart_send_blocking(USART2, k + '0');    /* USART2: Send byte. */
            gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
            uint32_t p = 800000 * k;
            for (i = 0; i < p; i++)    /* Wait a bit. */
                    __asm__("nop");
            k--;
        }
    }

    return 0;
}
