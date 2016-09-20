/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103

            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.

            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.

 */
#include <libopencm3/stm32/usart.h>

#ifndef STM32_DS18X20_ONEWIRE_H
#define STM32_DS18X20_ONEWIRE_H

#define WIRE_0	0x00
#define WIRE_1	0xff

void usart_enable_halfduplex(uint32_t usart);
void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity, uint32_t flowcontrol);
void bytetoBits(uint8_t ow_byte, uint8_t *bits);
uint8_t bitsToByte(uint8_t *bits);
int OneWireReset(uint32_t usart);
uint8_t OneWireSend(uint32_t usart, uint8_t command,
                    uint8_t *data, uint8_t dLength, uint8_t reset);



#endif //STM32_DS18X20_ONEWIRE_H
