#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>

#include <ow.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
OneWire ow_dev;

int _write(int file, char *ptr, int len);

/* STM32 в 72 MHz. */
static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOB, GPIOC, and AFIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable clocks for USARTs. */
    rcc_periph_clock_enable(RCC_USART2);
}

void usart2_isr() {
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        ow_bus_get_echo_data(&ow_dev, usart_recv_blocking(USART2));
    }
}


static void gpio_setup(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX | GPIO_USART2_RX);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX | GPIO_USART3_RX);

    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

    /* Preconf LED. */
    gpio_clear(GPIOC, GPIO13);
}

void usart_enable_halfduplex(uint32_t usart) {
    USART_CR2(usart) &= ~USART_CR2_LINEN;
    USART_CR2(usart) &= ~USART_CR2_CLKEN;
    USART_CR3(usart) &= ~USART_CR3_SCEN;
    USART_CR3(usart) &= ~USART_CR3_IREN;
    USART_CR3(usart) |= USART_CR3_HDSEL;
}

/** Метод реализует переключение выбранного USART в нужный режим
 * @param[in] baud Скорость в бодах (9600, 115200, etc...)
 */

void onewire_usart_setup(uint32_t baud) {
    nvic_disable_irq(NVIC_USART2_IRQ);
    usart_disable(USART2);

    // Настраиваем
    usart_set_baudrate(USART2, baud);
    usart_set_databits(USART2, 8);     // 8 bits
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable_halfduplex(USART2);

    usart_enable_rx_interrupt(USART2);
    usart_disable_tx_interrupt(USART2);
    //nvic_set_priority(NVIC_USART2_IRQ, 2);

    usart_enable(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
}

void onewire_send(uint16_t data) {
    usart_send_blocking(USART2, data);
    while (!(USART_SR(USART2) & USART_SR_TC));
}

int main(void) {

    clock_setup();
    gpio_setup();

    ow_dev.usart_setup = onewire_usart_setup;
    ow_dev.send = onewire_send;

    while (TRUE) {
        uint8_t rslt = ow_scan(&ow_dev);
        uint32_t pause = 0xffffff;
        while (pause--);
    }

    return 0;
}

#pragma clang diagnostic pop