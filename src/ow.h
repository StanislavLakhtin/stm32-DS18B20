/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103
            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Общая идея заключается в использовании аппаратного USART/UART uK для иммитации работы 1wire.
            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.
 */
#ifndef STM32_DS18X20_ONEWIRE_H
#define STM32_DS18X20_ONEWIRE_H

#include <libopencm3/cm3/common.h>

#define ONEWIRE_NOBODY 0xF0
#define ONEWIRE_SEARCH 0xF0
#define ONEWIRE_SKIP_ROM 0xCC
#define ONEWIRE_READ_ROM 0x33
#define ONEWIRE_MATCH_ROM 0x55
#define ONEWIRE_CONVERT_TEMPERATURE 0x44
#define ONEWIRE_READ_SCRATCHPAD 0xBE
#define ONEWIRE_WRITE_SCRATCHPAD 0x4E
#define ONEWIRE_COPY_SCRATCHPAD 0x48
#define ONEWIRE_RECALL_E2 0xB8

#ifndef ONEWIRE_MAXDEVICES_ON_THE_BUS
#define ONEWIRE_MAXDEVICES_ON_THE_BUS 10
#endif

#define DS18B20 0x28
#define DS18S20 0x10

#define WIRE_0    0x00 // 0x00 --default
#define WIRE_1    0xff
#define OW_READ   0xff

#ifndef FALSE
#define FALSE 0x00
#endif
#ifndef TRUE
#define TRUE  0x01
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    int8_t inCelsus;
    uint8_t frac;
} Temperature;

typedef struct {
    uint8_t family;
    uint8_t code[6];
    uint8_t crc;
} RomCode;

typedef struct {
    uint8_t crc;
    uint8_t reserved[3];
    uint8_t configuration;
    uint8_t tl;
    uint8_t th;
    uint8_t temp_msb;
    uint8_t temp_lsb;
} Scratchpad_DS18B20;

typedef struct {
    uint8_t crc;
    uint8_t count_per;
    uint8_t count_remain;
    uint8_t reserved[2];
    uint8_t tl;
    uint8_t th;
    uint8_t temp_msb;
    uint8_t temp_lsb;
} Scratchpad_DS18S20;

typedef void (*ow_send_fptr_t) (uint16_t data);
typedef void (*ow_usart_setup_fptr_t) (uint32_t baud);

typedef struct {
    volatile uint16_t rc_buffer;
    volatile uint8_t rc_flag;
    uint8_t ROM_BUFFER[8];
    uint8_t lastDiscrepancy;
    uint8_t lastFamilyDiscrepancy;
    uint8_t lastDeviceFlag;
    uint8_t hasMoreROM; // flag if MAXDEVICES_ON_THE_BUS is not enough
    uint8_t devicesQuantity;
    uint8_t crc8;
} OWState;

typedef struct {
    RomCode rom[ONEWIRE_MAXDEVICES_ON_THE_BUS];
    ow_send_fptr_t send;
    ow_usart_setup_fptr_t usart_setup;
    OWState state;
} OneWire;

void ow_clear_state(OneWire *ow_dev);
void ow_bus_get_echo_data(OneWire *ow_dev, uint16_t data);
uint16_t ow_read_blocking(OneWire * ow_dev);
uint16_t ow_reset_cmd(OneWire *ow_dev);
uint8_t ow_find_next_ROM(OneWire *ow_dev);
uint8_t ow_scan(OneWire * ow_dev);

#ifdef __cplusplus
}
#endif

#endif //STM32_DS18X20_ONEWIRE_H