/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103

            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.

            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.

 */
#ifndef STM32_DS18X20_ONEWIRE_H
#define STM32_DS18X20_ONEWIRE_H

#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define ONEWIRE_SEARCH 0xF0
#define ONEWIRE_SKIP_ROM 0xCC
#define ONEWIRE_READ_ROM 0x33
#define ONEWIRE_MATCH_ROM 0x55
#define ONEWIRE_CONVERT_TEMPERATURE 0x44
#define ONEWIRE_READ_SCRATCHPAD 0xBE

#ifndef MAXDEVICES_ON_THE_BUS
#define MAXDEVICES_ON_THE_BUS 5
#endif

#define WIRE_0    0x00
#define WIRE_1    0xff
#define OW_READ 0xff

//#define USART_CONSOLE USART2

volatile uint8_t recvFlag;
volatile uint16_t rc_buffer[5];

typedef struct {
    uint8_t crc;
    uint8_t code[6];
    uint8_t family;
} RomCode;

typedef struct {
    uint32_t usart;
    RomCode ids[MAXDEVICES_ON_THE_BUS];
} OneWire;

void usart_enable_halfduplex(uint32_t usart); /// вспомогательная функция по настройке HalfDuplex на USART
void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity,
                 uint32_t flowcontrol);

uint8_t getUsartIndex(uint32_t usart);

uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits);

void owInit(OneWire *ow);

uint16_t owResetCmd(OneWire *ow);

void owSearchCmd(OneWire *ow);

void owSkipRomCmd(OneWire *ow);

void owMatchRomCmd(OneWire *ow, RomCode *rom);

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom);

uint8_t* owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data);

void owSend(OneWire *ow, uint16_t data);

void owSendByte(OneWire *ow, uint8_t data);

uint16_t owEchoRead(OneWire *ow);


#ifdef ONEWIRE_UART5
void usart3_isr(void) {
    /* Проверяем, что мы вызвали прерывание из-за RXNE. */
    if (((USART_CR1(UART5) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(UART5) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        rc_buffer[4] = usart_recv_blocking(UART5);
        recvFlag &= ~(1 << 4);
    }
}
#endif
#ifdef ONEWIRE_UART4
void uart4_isr(void) {
    /* Проверяем, что мы вызвали прерывание из-за RXNE. */
    if (((USART_CR1(UART4) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(UART4) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        rc_buffer[3] = usart_recv_blocking(UART4);
        recvFlag &= ~(1 << 3);
    }
}
#endif
#ifdef ONEWIRE_USART3
void usart3_isr(void) {
    /* Проверяем, что мы вызвали прерывание из-за RXNE. */
    if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        rc_buffer[2] = usart_recv_blocking(USART3);
        recvFlag &= ~(1 << 2);
    }
}
#endif
#ifdef ONEWIRE_USART2
void usart2_isr(void) {
    /* Проверяем, что мы вызвали прерывание из-за RXNE. */
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        rc_buffer[1] = usart_recv_blocking(USART3);
        recvFlag &= ~(1 << 1);
    }
}
#endif
#ifdef ONEWIRE_USART1
void usart1_isr(void) {
    /* Проверяем, что мы вызвали прерывание из-за RXNE. */
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        rc_buffer[0] = usart_recv_blocking(USART3);
        recvFlag &= ~1;
    }
}
#endif
#endif //STM32_DS18X20_ONEWIRE_H
