#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdio.h>

namespace Config {
    constexpr uint32_t pruBaseFreq = 120000;        // PRU Base thread ISR update frequency (hz)
    constexpr uint32_t pruServoFreq = 1000;        // PRU Servo thread ISR update frequency (hz)
    constexpr uint32_t oversample = 3;
    constexpr uint32_t swBaudRate = 19200;         // Software serial baud rate
    constexpr uint32_t pruSerialFreq = swBaudRate * oversample;

    constexpr uint32_t stepBit = 22;               // Bit location in DDS accum
    constexpr uint32_t stepMask = (1L << stepBit);

    constexpr uint32_t joints = 8;                 // Number of joints
    constexpr uint32_t variables = 6;              // Number of command values

    constexpr uint32_t pruData = 0x64617461;       // "data" SPI payload
    constexpr uint32_t pruRead = 0x72656164;       // "read" SPI payload
    constexpr uint32_t pruWrite = 0x77726974;      // "writ" SPI payload
    constexpr uint32_t pruEstop = 0x65737470;      // "estp" SPI payload
    constexpr uint32_t pruAcknowledge = 0x61636b6e;// "ackn" SPI payload
    constexpr uint32_t pruErr = 0x6572726f;        // "erro" payload

    // IRQ priorities
    constexpr uint32_t baseThreadIrqPriority = 1;
    constexpr uint32_t servoThreadIrqPriority = 2;
    constexpr uint32_t serialThreadIrqPriority = 3;
    constexpr uint32_t spiDmaTxIrqPriority = 4;
    constexpr uint32_t spiDmaRxIrqPriority = 5;
    constexpr uint32_t spiNssIrqPriority = 6;

    // Serial configuration
    constexpr uint32_t pcBaud = 115200;            // UART baudrate

    constexpr uint32_t dataErrMax = 100;

    // SPI configuration
    constexpr uint32_t dataBuffSize = 64;          // Size of SPI receive buffer
}

#endif
