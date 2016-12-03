#include "OneWire.h"

/**
    @author Stanislav Lakhtin
    @date   11.07.2016
    @brief  Реализация протокола 1wire на базе библиотеки libopencm3 для микроконтроллера STM32F103

            Возможно, библиотека будет корректно работать и на других uK (требуется проверка).
            Общая идея заключается в использовании аппаратного USART uK для иммитации работы 1wire.

            Подключение устройств осуществляется на выбранный USART к TX пину, который должен быть подтянут к линии питания сопротивлением 4.7К.
            Реализация библиотеки осуществляет замыкание RX на TX внутри uK, оставляя ножку RX доступной для использования в других задачах.

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
 */
void usart_setup(uint32_t usart, uint32_t baud, uint32_t bits, uint32_t stopbits, uint32_t mode, uint32_t parity,
                 uint32_t flowcontrol) {
    uint8_t irqNumber = NVIC_USAGE_FAULT_IRQ;
    switch (usart) {
        case (USART1):
            irqNumber = NVIC_USART1_IRQ;
            break;
        case (USART2):
            irqNumber = NVIC_USART2_IRQ;
            break;
        case (USART3):
            irqNumber = NVIC_USART3_IRQ;
            break;
        case (UART4):
            irqNumber = NVIC_UART4_IRQ;
            break;
        case (UART5):
            irqNumber = NVIC_UART5_IRQ;
            break;
        default:
            return;
    }
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
    int i, k = 0;
    for (i = 0; i < MAXDEVICES_ON_THE_BUS; i++) {
        ow->ids[i].crc = 0x00;
        ow->ids[i].family = 0x00;
        for (k = 0; k < 6; k++)
            ow->ids[k].code[k] = 0x00;
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
    }
    return -1;
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
    while (recvFlag & (1 << i));
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

int8_t owCRC(uint8_t crc, uint8_t b) {
    for (uint8_t p = 8; p; p--) {
        crc = ((crc ^ b) & 1) ? (crc >> 1) ^ 0b10001100 : (crc >> 1);
        b >>= 1;
    }
    return crc;
}

/**
 * Method for SEARCH any devices on the bus and put it on the ow->ids
 * If MAXDEVICE_ON_THE_BUS smaller then count of real devices
 * @param ow -- OneWire pointer
 */
void owSearchCmd(OneWire *ow) {
    uint8_t devNum = 0;
    int8_t forkBite = -1;
    bool oneMoreDevice = true;
    uint8_t *readBuffer; // Здесь будет накапливаться побитно ROM ID очередного устройства
    //очищаем все ранее найденные устройства
    owInit(ow);
    while (devNum < MAXDEVICES_ON_THE_BUS && oneMoreDevice) {
        oneMoreDevice = false;
        owResetCmd(ow);
        // посылка команды ОЧЕРЕДНОГО устройства на поиск
        owSendByte(ow, ONEWIRE_SEARCH);
        // будем двигаться от МЛАДШЕГО БИТА К СТАРШЕМУ до тех пор, пока не достигнем старшего или пока не достигнем
        // максимально-возможного количества устройств. Если устройств больше, то в соответствии с логикой работы
        // будут найдены столько, сколько было определено MAXDEVICES_ON_THE_BUS
        // стараемся загрузить 64 бит [FAMILY CODE(1B)][ROM CODE(6B)][CRC(1B)] (ОБРАТНЫЙ ПОРЯОК БИТ)
        uint8_t bC = 0;
        while (bC < 64) {
            readBuffer = ((uint8_t *) (&ow->ids[devNum]) + 7 - bC / 8);
            // в соответствии с логикой читаем бит посылая два цикла чтения
            // если пришёл конфликтный бит, то принимаем всегда за ноль и продолжаем опрос
            uint8_t cB, cB_inverse, sB;
            owSend(ow, OW_READ); // чтение прямого бита
            cB = owReadSlot(owEchoRead(ow));
            owSend(ow, OW_READ); // чтение инверсного бита
            cB_inverse = owReadSlot(owEchoRead(ow));
            if ((cB == cB_inverse)) {
                // был конфликт -- биты НЕ совпали у нескольких устройств
                // в этм месте УЖЕ произошёл fork
                sB = (forkBite == bC) ? 1 : 0; // мы находимся в режиме разрешения предыдущего конфликта или в новом?
                if (sB == 0) {
                    forkBite = bC;
                    oneMoreDevice = true;
                } else {
                    oneMoreDevice = false;
                }
            } else {
                // если прямой и инверсный биты разные, то всё ок. Просто запоминаем, что пришло
                sB = cB;
            }
            // сохраняем бит
            if (sB == 1)
                *(readBuffer) |= 1 << bC % 8;
            uint8_t answerBit = (sB == 0) ? WIRE_0 : WIRE_1;
            owSend(ow, answerBit);
            //*(((uint8_t *) &ow->ids[devNum]) + 7 - bC) = readBuffer;
            bC++;
        }
        uint8_t crcCheck = 0x00;
        int i = 7;
        for (; i > 0; i--)
            crcCheck = owCRC(crcCheck, *(((uint8_t *) &ow->ids[devNum]) + i)); //todo что делать, если не получился 0?
        devNum++;
    }
}

void owSkipRomCmd(OneWire *ow) {
    owResetCmd(ow);
    owSendByte(ow, ONEWIRE_SKIP_ROM);
}

void owMatchRomCmd(OneWire *ow, RomCode *rom) {
    owResetCmd(ow);
    owSendByte(ow, ONEWIRE_MATCH_ROM);
    int i = 7;
    for (; i >= 0; i--)
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
    int8_t sign;
    uint8_t pad[9];
    Scratchpad_DS18B20 *sp = (Scratchpad_DS18B20 *)&pad;
    Scratchpad_DS18S20 *spP = (Scratchpad_DS18S20 *)&pad;
    switch (rom->family) {
        case DS18B20:
            owReadScratchpadCmd(ow, rom, pad);
            sign = (int8_t) ((sp->temp_msb & 0xf8) == 0x00 ? 1 : -1);
            t.inCelsus = sign * (((sp->temp_msb & 0x07) << 4) |
                         ((sp->temp_lsb >> 4) & 0x0f));
            t.frac = (uint8_t) ((((sp->temp_lsb & 0x0F)) * 10) >> 4);
            break;
        case DS18S20:
            owReadScratchpadCmd(ow, rom, pad);
            sign = (int8_t) ((spP->temp_msb & 0xff) == 0x00 ? 1 : -1);
            t.inCelsus = sign * ((spP->temp_lsb >> 1) & 0x7f);
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