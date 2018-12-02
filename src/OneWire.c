#include "OneWire.h"
#include <stdio.h>

/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103

            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Проверка необходима, чтобы убедиться в корректности настройки UART/USART для работы
            в полудуплексном режиме
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.

            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.

            Реализация библиотеки предполагает возможную одновременную работу как с независимыми шинами сразу со всеми
            возможными UART/USART в микроконтроллере. При этом все шины (до 5 штук) будут адресоваться и опрашиваться индивидуально
 */

/// Метод реализует переключение работы USART в half-duplex режим. Метод не работает для 1wire реализации
void usart_enable_halfduplex(uint32_t usart) {
    USART_CR2(usart) &= ~USART_CR2_LINEN;
    USART_CR2(usart) &= ~USART_CR2_CLKEN;
    USART_CR3(usart) &= ~USART_CR3_SCEN;
    USART_CR3(usart) &= ~USART_CR3_IREN;
    USART_CR3(usart) |= USART_CR3_HDSEL;
}

/** Метод реализует переключение выбранного USART в нужный режим
 * @param[in] usart Выбранный аппаратный usart -- (USART1, USART2, etc...)
 * @param[in] baud Скорость в бодах (9600, 115200, etc...)
 * @param[in] bits Биты данных (8,9)
 * @param[in] stopbits СтопБиты (USART_STOPBITS_1, USART_STOPBITS_0)
 * @param[in] mode Режим работы (USART_MODE_TX_RX, etc...)
 * @param[in] parity Контроль чётности (USART_PARITY_NONE, etc...)
 * @param[in] flowcontrol Управление потоком (USART_FLOWCONTROL_NONE, etc...)
 *
 */

uint8_t getUsartIndex(uint32_t usart);

void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity,
                 uint32_t flowcontrol) {
    uint8_t irqs[] = {NVIC_USART1_IRQ, NVIC_USART2_IRQ, NVIC_USART3_IRQ, NVIC_UART4_IRQ, NVIC_UART5_IRQ};
    uint8_t irqNumber = irqs[getUsartIndex(usart)];

    nvic_disable_irq(irqNumber);
    usart_disable(usart);

    // Настраиваем
    usart_set_baudrate(usart, baud);
    usart_set_databits(usart, bits);
    usart_set_stopbits(usart, stopbits);
    usart_set_mode(usart, mode);
    usart_set_parity(usart, parity);
    usart_set_flow_control(usart, flowcontrol);

    usart_enable_rx_interrupt(usart);

    usart_enable_halfduplex(usart);
    usart_enable(usart);
    nvic_enable_irq(irqNumber);
}

void owInit(OneWire *ow) {
    int i = 0, k = 0;
    for (; i < MAXDEVICES_ON_THE_BUS; i++) {
        uint8_t *r = &ow->ids[i];
        k = 0;
        for (; k < 8; k++)
            r[k] = 0;
    }
    k = 0;
    for (; k < 8; k++)
        ow->lastROM[k] = 0x00;
    ow->lastDiscrepancy = 64;

}

void owReadHandler(uint32_t usart) {
    uint8_t index = getUsartIndex(usart);
    /* Проверяем, что мы вызвали прерывание из-за RXNE. */
    if (((USART_CR1(usart) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(usart) & USART_SR_RXNE) != 0)) {

        /* Получаем данные из периферии и сбрасываем флаг*/
        rc_buffer[index] = usart_recv_blocking(usart);
        recvFlag &= ~(1 << index);
    }
}

/** Реализация RESET на шине 1wire
 *
 * @param usart -- выбранный для реализации 1wire usart
 * @return Возвращает 1 если на шине кто-то есть и 0 в противном случае
 */

uint16_t owResetCmd(OneWire *ow) {
    usart_setup(ow->usart, 9600, 8, USART_STOPBITS_1, USART_MODE_TX_RX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);

    owSend(ow, 0xF0); // Send RESET
    uint16_t owPresence = owEchoRead(ow); // Ждём PRESENCE на шине и вовзращаем, что есть

    usart_setup(ow->usart, 115200, 8, USART_STOPBITS_1, USART_MODE_TX_RX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    return owPresence;
}

uint8_t getUsartIndex(uint32_t usart) {
    switch (usart) {
        case (USART1):
            return 0;
        case (USART2):
            return 1;
        case (USART3):
            return 2;
        case (UART4):
            return 3;
        case (UART5):
            return 4;
        default:
            return getUsartIndex(USART3);
    }
}

void owSend(OneWire *ow, uint16_t data) {
    recvFlag |= (1 << getUsartIndex(ow->usart));
    usart_send(ow->usart, data);
    while (!usart_get_flag(ow->usart, USART_SR_TC));
}

uint8_t owReadSlot(uint16_t data) {
    return (data == OW_READ) ? 1 : 0;
}

uint16_t owEchoRead(OneWire *ow) {
    uint8_t i = getUsartIndex(ow->usart);
    uint16_t pause = 1000;
    while (recvFlag & (1 << i) && pause--);
    return rc_buffer[i];
}

uint8_t *byteToBits(uint8_t ow_byte, uint8_t *bits) {
    uint8_t i;
    for (i = 0; i < 8; i++) {
        if (ow_byte & 0x01) {
            *bits = WIRE_1;
        } else {
            *bits = WIRE_0;
        }
        bits++;
        ow_byte = ow_byte >> 1;
    }
    return bits;
}

/**
 * Метод пересылает последовательно 8 байт по одному на каждый бит в data
 * @param usart -- выбранный для эмуляции 1wire USART
 * @param d -- данные
 */
void owSendByte(OneWire *ow, uint8_t d) {
    uint8_t data[8];
    byteToBits(d, data);
    int i;
    for (i = 0; i < 8; ++i) {
        owSend(ow, data[i]);
    }
}


uint8_t bitsToByte(uint8_t *bits) {
    uint8_t target_byte, i;
    target_byte = 0;
    for (i = 0; i < 8; i++) {
        target_byte = target_byte >> 1;
        if (*bits == WIRE_1) {
            target_byte |= 0x80;
        }
        bits++;
    }
    return target_byte;
}

/* Подсчет CRC8 массива mas длиной Len */
uint8_t owCRC(uint8_t *mas, uint8_t Len) {
    uint8_t i, dat, crc, fb, st_byt;
    st_byt = 0;
    crc = 0;
    do {
        dat = mas[st_byt];
        for (i = 0; i < 8; i++) {  // счетчик битов в байте
            fb = crc ^ dat;
            fb &= 1;
            crc >>= 1;
            dat >>= 1;
            if (fb == 1) crc ^= 0x8c; // полином
        }
        st_byt++;
    } while (st_byt < Len); // счетчик байтов в массиве
    return crc;
}

uint8_t owCRC8(RomCode *rom) {
    return owCRC(rom, 7);
}

/*
 * return 1 if has got one more address
 * return 0 if hasn't
 * return -1 if error reading happened
 *
 * переделать на функции обратного вызова для реакции на ошибки
 */
int hasNextRom(OneWire *ow, uint8_t *ROM) {
    if (owResetCmd(ow) == ONEWIRE_NOBODY) {
        return 0;
    }

    owSendByte(ow, ONEWIRE_SEARCH);
    uint8_t ui32BitNumber = 0;
    int zeroFork = -1;
    do {
        int byteNum = ui32BitNumber / 8;
        uint8_t *current = (ROM) + byteNum;
        uint8_t cB, cmp_cB, searchDirection = 0;
        owSend(ow, OW_READ); // чтение прямого бита
        cB = owReadSlot(owEchoRead(ow));
        owSend(ow, OW_READ); // чтение инверсного бита
        cmp_cB = owReadSlot(owEchoRead(ow));
        if (cB == cmp_cB && cB == 1)
            return -1;
        if (cB != cmp_cB) {
            searchDirection = cB;
        } else {
            if (ui32BitNumber == ow->lastDiscrepancy)
                searchDirection = 1;
            else {
                if (ui32BitNumber > ow->lastDiscrepancy) {
                    searchDirection = 0;
                } else {
                    searchDirection = (uint8_t) ((ow->lastROM[byteNum] >> ui32BitNumber % 8) & 0x01);
                }
                if (searchDirection == 0)
                    zeroFork = ui32BitNumber;
            }
        }
        // сохраняем бит
        if (searchDirection)
            *(current) |= 1 << ui32BitNumber % 8;
        uint8_t answerBit = (uint8_t) ((searchDirection == 0) ? WIRE_0 : WIRE_1);
        owSend(ow, answerBit);
        ui32BitNumber++;
    } while (ui32BitNumber < 64);
    ow->lastDiscrepancy = zeroFork;
    uint8_t i = 0;
    for (; i < 7; i++)
        ow->lastROM[i] = ROM[i];
    return ow->lastDiscrepancy > 0;
}

// Возвращает количество устройств на шине или код ошибки, если значение меньше 0
int owSearchCmd(OneWire *ow) {
    int device = 0, nextROM;
    owInit(ow);
    do {
        nextROM = hasNextRom(ow, &ow->ids[device]);
        if (nextROM < 0)
            return -1;
        device++;
    } while (nextROM && device < MAXDEVICES_ON_THE_BUS);
    return device;
}

void owSkipRomCmd(OneWire *ow) {
    owResetCmd(ow);
    owSendByte(ow, ONEWIRE_SKIP_ROM);
}

void owMatchRomCmd(OneWire *ow, RomCode *rom) {
    owResetCmd(ow);
    owSendByte(ow, ONEWIRE_MATCH_ROM);
    int i = 0;
    for (; i < 8; i++)
        owSendByte(ow, *(((uint8_t *) rom) + i));
}

void owConvertTemperatureCmd(OneWire *ow, RomCode *rom) {
    owMatchRomCmd(ow, rom);
    owSendByte(ow, ONEWIRE_CONVERT_TEMPERATURE);
}

/**
 * Method for reading scratchad DS18B20 OR DS18S20
 * If sensor DS18B20 then data MUST be at least 9 byte
 * If sensor DS18S20 then data MUST be at least 2 byte
 * @param ow -- OneWire pointer
 * @param rom -- selected device on the bus
 * @param data -- buffer for data
 * @return data
 */
uint8_t *owReadScratchpadCmd(OneWire *ow, RomCode *rom, uint8_t *data) {
    uint16_t b = 0, p;
    switch (rom->family) {
        case DS18B20:
        case DS18S20:
            p = 72;
            break;
        default:
            return data;

    }
    owMatchRomCmd(ow, rom);
    owSendByte(ow, ONEWIRE_READ_SCRATCHPAD);
    while (b < p) {
        uint8_t pos = (uint8_t) ((p - 8) / 8 - (b / 8));
        owSend(ow, OW_READ);
        uint8_t bt = owReadSlot(owEchoRead(ow));

        if (bt == 1)
            data[pos] |= 1 << b % 8;
        else
            data[pos] &= ~(1 << b % 8);
        b++;
    }
    return data;
}

void owWriteDS18B20Scratchpad(OneWire *ow, RomCode *rom, uint8_t th, uint8_t tl, uint8_t conf) {
    if (rom->family != DS18B20)
        return;
    owMatchRomCmd(ow, rom);
    owSendByte(ow, ONEWIRE_WRITE_SCRATCHPAD);
    owSendByte(ow, th);
    owSendByte(ow, tl);
    owSendByte(ow, conf);
}

/**
 * Get last mesaured temperature from DS18B20 or DS18S20. These temperature MUST be measured in previous
 * opearions. If you want to measure new value you can set reSense in true. In this case next invocation
 * that method will return value calculated in that step.
 * @param ow -- OneWire bus pointer
 * @param rom -- selected device
 * @param reSense -- do you want resense temp for next time?
 * @return struct with data
 */
Temperature readTemperature(OneWire *ow, RomCode *rom, bool reSense) {
    Temperature t;
    t.inCelsus = 0x00;
    t.frac = 0x00;
    uint8_t pad[9];
    Scratchpad_DS18B20 *sp = (Scratchpad_DS18B20 *) &pad;
    Scratchpad_DS18S20 *spP = (Scratchpad_DS18S20 *) &pad;
    switch (rom->family) {
        case DS18B20:
            owReadScratchpadCmd(ow, rom, pad);
            t.inCelsus = (int8_t) (sp->temp_msb << 4) |
                         (sp->temp_lsb >> 4);
            t.frac = (uint8_t) ((((sp->temp_lsb & 0x0F)) * 10) >> 4);
            break;
        case DS18S20:
            owReadScratchpadCmd(ow, rom, pad);
            t.inCelsus = spP->temp_lsb >> 1;
            t.frac = (uint8_t) 5 * (spP->temp_lsb & 0x01);
            break;
        default:
            return t;
    }
    if (reSense) {
        owConvertTemperatureCmd(ow, rom);
    }
    return t;
}

void owCopyScratchpadCmd(OneWire *ow, RomCode *rom) {
    owMatchRomCmd(ow, rom);
    owSendByte(ow, ONEWIRE_COPY_SCRATCHPAD);
}

void owRecallE2Cmd(OneWire *ow, RomCode *rom) {
    owMatchRomCmd(ow, rom);
    owSendByte(ow, ONEWIRE_RECALL_E2);
}