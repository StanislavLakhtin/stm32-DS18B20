#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/common/usart_common_v2.h>
#include <stdio.h>

/* Set STM32 to 72 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable GPIOB, GPIOC, and AFIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_AFIO);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART2);
}

static void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity, uint32_t flowcontrol)
{
    /* Setup parameters. */
    usart_set_baudrate(usart, baud);
    usart_set_databits(usart, bits);
    usart_set_stopbits(usart, stopbits);
    usart_set_mode(usart, mode);
    usart_set_parity(usart, parity);
    usart_set_flow_control(usart, flowcontrol);

    /* Finally enable the USART. */
    //usart_enable(usart);
    usart_enable_halfduplex(usart);
}

static void gpio_setup(void)
{
    /* Setup GPIO pin GPIO_USART2_TX. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

    /* Set GPIO8 (in GPIO port A) to 'output push-pull'. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);

	/* Set GPIO13 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	/* Set GPIO4 (in GPIO port B) to 'output push-pull'. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);

	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

	/* Preconfigure the LEDs. */
	gpio_set(GPIOB, GPIO4);	/* Switch off LED. */
	gpio_clear(GPIOC, GPIO13);	/* Switch on LED. */
}

int main(void)
{
    clock_setup();
	gpio_setup();
    usart_setup(USART2, 9600, 8, USART_STOPBITS_1, USART_MODE_TX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);

    int i;
	/* Blink the LEDs (PC13 and PB4) on the board. */
	while (1) {
        int k = 10;
        while (k>0) {
            usart_send_blocking(USART2, k + '0');	/* USART2: Send byte. */
            gpio_toggle(GPIOC, GPIO13);    /* LED on/off */
            gpio_toggle(GPIOB, GPIO4);    /* LED on/off */
            uint32_t p = 800000*k;
            for (i = 0; i < p; i++)    /* Wait a bit. */
               __asm__("nop");
            k--;
        }
	}

	return 0;
}
