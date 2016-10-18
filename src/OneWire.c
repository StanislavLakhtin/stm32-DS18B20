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
    uint8_t irqNumber = NVIC_USART1_IRQ;
    switch (usart) {
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

uint16_t owReset(OneWire *ow) {
    usart_setup(ow->usart, 9600, 8, USART_STOPBITS_1, USART_MODE_TX_RX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);

    owSend(ow, 0xF0); // Send RESET
    uint16_t oneWireDevices = owEchoRead(ow); // Ждём PRESENCE на шине

    usart_setup(ow->usart, 115200, 8, USART_STOPBITS_1, USART_MODE_TX_RX, USART_PARITY_NONE, USART_FLOWCONTROL_NONE);
    return oneWireDevices;
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


/**
 * Метод поиска устройств на шине 1Wire
 * Если были возвращены все возможные устройства, циклически возвращается первое
 */
void owSearchCmd(OneWire *ow) {
    uint8_t devNum = 0;
    int8_t forkBite = -1;
    bool oneMoreDevice = true;
    uint8_t *readBuffer; // Здесь будет накапливаться побитно ROM ID очередного устройства
    //очищаем все ранее найденные устройства
    owInit(ow);
    while (devNum < MAXDEVICES_ON_THE_BUS && oneMoreDevice) {
        owReset(ow);
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
            else
                *(readBuffer) &= ~(1 << bC % 8);
            uint8_t answerBit = (sB == 0) ? WIRE_0 : WIRE_1;
            owSend(ow, answerBit);
            //*(((uint8_t *) &ow->ids[devNum]) + 7 - bC) = readBuffer;
            bC++;
        }
        devNum++;
    }
}

#ifdef _STDIO_H_ //should be #include <stdio.h> & <errno.h> & #define USART_CONSOLE to use printf
int _write(int file, char *ptr, int len) {
    int i;

    if (file == 1) {
        for (i = 0; i < len; i++)
            usart_send_blocking(USART_CONSOLE, ptr[i]);
        return i;
    }
    errno = EIO;
    return -1;
}
#endif