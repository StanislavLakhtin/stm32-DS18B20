#include <ow.h>

/**
    @author Stanislav Lakhtin
    @date   12.07.2019
    @brief  OneWire библиотека для микроконтроллеров на базе STM32 с использованием UART/USART
            Программное обеспеение предоставляется как есть, без каких либо гарантий.
            Основной поисковый алгоритм реализован с использованием демонстрационного кода и эталонного
            кода реализации компании MAXIM

            В случае использования, просьба ссылаться на первоисточник.
 */

static uint8_t docrc8(OneWire* ow_dev, uint8_t value);

void ow_clear_state(OneWire *ow_dev) {
    OWState * state = &ow_dev->state;
    state->hasMoreROM = FALSE;
    state->lastDiscrepancy = 0x00;
    state->lastFamilyDiscrepancy = 0x00;
    state->lastDeviceFlag = FALSE;
    state->rc_flag = FALSE;
    state->rc_buffer = 0x00;
    state->devicesQuantity = 0x00;
    state->crc8 = 0x00;
    for (uint8_t i = 0; i < ONEWIRE_MAXDEVICES_ON_THE_BUS; i++) {
        RomCode * ROM = &ow_dev->rom[i];
        ROM->crc = 0x00;
        ROM->family = 0x00;
        for (uint8_t n = 0; n < 6; n++ )
            ROM->code[n] = 0x00;
    }
}

uint16_t ow_reset_cmd(OneWire *ow_dev) {
    ow_dev->usart_setup(9600);

    ow_dev->send(0xF0); // RESET
    uint16_t owPresence = ow_read_blocking(ow_dev); // Ждём PRESENCE на шине и вовзращаем, что есть

    ow_dev->usart_setup(115200);
    return owPresence;
}

void ow_bus_get_echo_data(OneWire *ow_dev, uint16_t data) {
    ow_dev->state.rc_buffer = data;
    ow_dev->state.rc_flag = TRUE;
}

uint16_t ow_read_blocking(OneWire * ow_dev) {
    while ( !ow_dev->state.rc_flag );
    ow_dev->state.rc_flag = FALSE;
    return ow_dev->state.rc_buffer;
}

uint8_t ow_read_bit(OneWire *ow_dev) {
    ow_dev->send(OW_READ);
    return ow_read_blocking(ow_dev) == OW_READ ? 0x01 : 0x00;
}

uint8_t ow_scan(OneWire *ow_dev) {
    uint8_t rslt = FALSE, num=0;
    ow_clear_state(ow_dev);
    do {
        rslt = ow_find_next_ROM(ow_dev);
        if ( rslt == TRUE ) {
            // нашёлся очередной ROM на шине
            if ( num < ONEWIRE_MAXDEVICES_ON_THE_BUS ) {
                ow_dev->rom[num].family = ow_dev->state.ROM_BUFFER[0];
                ow_dev->rom[num].crc = ow_dev->state.ROM_BUFFER[7];
                for (uint8_t n=1; n < 7; n++)
                    ow_dev->rom[num].code[n-1] = ow_dev->state.ROM_BUFFER[n];
                ow_dev->state.devicesQuantity += 1;
            } else {
                // сохраняем флаг о том, что есть ROM на шине, который не влез в нашу таблицу и выходим с ошибкой
                ow_dev->state.hasMoreROM = TRUE;
                return FALSE;
            }
            num += 1;
        }
    } while ( rslt && !ow_dev->state.lastDeviceFlag);
    return TRUE;
}

uint8_t ow_find_next_ROM(OneWire *ow_dev) {
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number, search_result;
    uint8_t id_bit, cmp_id_bit;
    uint8_t rom_byte_mask, search_direction;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    OWState *state = &ow_dev->state;

    // если крайний вызов был не для крайнего устройства на шине
    if (!state->lastDeviceFlag) {
        // 1-Wire reset
        if (ow_reset_cmd(ow_dev) == ONEWIRE_NOBODY) {
            // сброс поиска
            state->lastDiscrepancy = 0;
            state->lastDeviceFlag = FALSE;
            state->lastFamilyDiscrepancy = 0;
            return FALSE;
        }

        ow_dev->send(ONEWIRE_SEARCH);

        do {
            // чтение прямого и комплиментарного битов
            id_bit = ow_read_bit(ow_dev);
            cmp_id_bit = ow_read_bit(ow_dev);

            // проверка, что на шине нет ни одного устрйоства. В этом случае и прямой и комплиментарный биты равны
            if ( id_bit && cmp_id_bit )
                break;
            else {
                // все устройства имеют одинаковый ответ: 0 or 1, не важно сейчас
                if (id_bit != cmp_id_bit)
                    search_direction = id_bit;  // выбираем в качестве "направления" прямой бит
                else {
                    // В случсе несоответствия (у некоторых устройств 0, а у других 1), проверяем, чтобы несоответствие было
                    // ДО зафиксированного положения в Last Discrepancy
                    // сдеанного на предыдущем шаге
                    if (id_bit_number < state->lastDiscrepancy)
                        search_direction = ((state->ROM_BUFFER[rom_byte_number] & rom_byte_mask) > 0);
                    else
                        // если равны в последнем сравнении, выбираем 1, или 0 в противном случае
                        search_direction = (id_bit_number == state->lastDiscrepancy);

                    // Если было 0, то записываем эту позицию в LastZero
                    if (search_direction == 0) {
                        last_zero = id_bit_number;

                        // Проверяем, чтобы last_zero было последней в выборе семейства устройств
                        if (last_zero < 9)
                            state->lastFamilyDiscrepancy = last_zero;
                    }
                }

                // устанавливаем (или стираем) бит в ROM
                // с масской rom_byte_mask
                if (search_direction)
                    state->ROM_BUFFER[rom_byte_number] |= rom_byte_mask;
                else
                    state->ROM_BUFFER[rom_byte_number] &= ~rom_byte_mask;

                // отсылаем на шину выбранное нами направление сканирования
                uint8_t answerBit = (uint8_t) ((search_direction == 0) ? WIRE_0 : WIRE_1);
                ow_dev->send(answerBit);

                // выполняем инкремент бита id_bit_number
                // и сдвигаем маску rom_byte_mask
                id_bit_number += 1;
                rom_byte_mask <<= 1;

                // Если маска установлена в 0 идём в новый байт SerialNum rom_byte_number и сбрасываем маску
                if (rom_byte_mask == 0) {
                    docrc8(ow_dev, state->ROM_BUFFER[rom_byte_number]);  // походу рассчитываем CRC
                    rom_byte_number += 1;
                    rom_byte_mask = 0x01;
                }
            }
        } while (rom_byte_number < 8);  // сканировать будем все ROM байты (от 0-7)

        // Если поиск был успешным
        if (!((id_bit_number < 65) || (state->crc8 != 0))) {
            // считаем, что поиск очередного ROM прошёл успешно и устанавливаем LastDiscrepancy, LastDeviceFlag, search_result
            state->lastDiscrepancy = last_zero;

            // проверяем, является ли результат поиска последним на шине
            if (state->lastDiscrepancy == 0)
                state->lastDeviceFlag = TRUE;

            search_result = TRUE;
        }
    }

    // Если не было найдено ни одного устройства на шине :-( то мы считаем, что шина чистая и следующий поиск будет как в первый раз
    if (!search_result || !state->ROM_BUFFER[0]) {
        ow_clear_state(ow_dev);
        search_result = FALSE;
    }

    return search_result;
}

static unsigned char dscrc_table[] = {
        0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
        157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
        35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
        190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
        70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
        219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
        101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
        248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
        140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
        17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
        175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
        50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
        202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
        87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
        233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
        116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

static uint8_t docrc8(OneWire* ow_dev, uint8_t value) {
    ow_dev->state.crc8 = dscrc_table[ow_dev->state.crc8 ^ value];
    return ow_dev->state.crc8;
}