/*

EspBitBanger.cpp - Implementation of a bit banger for proprietary serial protocols.
Copyright (c) 2023 David Wischnjak.

Based upon EspSoftwareSerial by:
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.
Copyright (c) 2018-2019 Dirk O. Kaar. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "EspBitBanger.h"
#include <Arduino.h>

#ifndef ESP32
uint32_t EspBitBanger::m_savedPS = 0;
#else
portMUX_TYPE EspBitBanger::m_interruptsMux = portMUX_INITIALIZER_UNLOCKED;
#endif

inline void IRAM_ATTR EspBitBanger::disableInterrupts()
{
#ifndef ESP32
    m_savedPS = xt_rsil(15);
#else
    taskENTER_CRITICAL(&m_interruptsMux);
#endif
}

inline void IRAM_ATTR EspBitBanger::restoreInterrupts()
{
#ifndef ESP32
    xt_wsr_ps(m_savedPS);
#else
    taskEXIT_CRITICAL(&m_interruptsMux);
#endif
}

constexpr uint8_t BYTE_ALL_BITS_SET = ~static_cast<uint8_t>(0);

EspBitBanger::EspBitBanger() {
    m_isrOverflow = false;
    m_rxGPIOPullUpEnabled = true;
    m_txGPIOOpenDrain = false;
}

EspBitBanger::EspBitBanger(int8_t rxPin, int8_t txPin)
{
    m_isrOverflow = false;
    m_rxGPIOPullUpEnabled = true;
    m_txGPIOOpenDrain = false;
    m_rxPin = rxPin;
    m_txPin = txPin;
}

EspBitBanger::~EspBitBanger() {
    end();
}

constexpr bool EspBitBanger::isValidGPIOpin(int8_t pin) {
#if defined(ESP8266)
    return (pin >= 0 && pin <= 16) && !isFlashInterfacePin(pin);
#elif defined(ESP32)
    // Remove the strapping pins as defined in the datasheets, they affect bootup and other critical operations
    // Remmove the flash memory pins on related devices, since using these causes memory access issues.
#ifdef CONFIG_IDF_TARGET_ESP32
    // Datasheet https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf,
    // Pinout    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/esp32-devkitC-v4-pinout.jpg
    return (pin == 1) || (pin == 2) || (pin >= 3 && pin <= 5) ||
        (pin >= 12 && pin <= 15) ||
        (!psramFound() && pin >= 16 && pin <= 17) ||
        (pin >= 18 && pin <= 19) ||
        (pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) || (pin >= 32 && pin <= 39);
#elif CONFIG_IDF_TARGET_ESP32S2
    // Datasheet https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf,
    // Pinout    https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/_images/esp32-s2_saola1-pinout.jpg
    return (pin >= 1 && pin <= 21) || (pin >= 33 && pin <= 44);
#elif CONFIG_IDF_TARGET_ESP32C3
    // Datasheet https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf,
    // Pinout    https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/_images/esp32-c3-devkitm-1-v1-pinout.jpg
    return (pin >= 0 && pin <= 1) || (pin >= 3 && pin <= 7) || (pin >= 18 && pin <= 21);
#else
    return pin >= 0;
#endif
#else
    return pin >= 0;
#endif
}

constexpr bool EspBitBanger::isValidRxGPIOpin(int8_t pin) {
    return isValidGPIOpin(pin)
#if defined(ESP8266)
        && (pin != 16)
#endif
        ;
}

constexpr bool EspBitBanger::isValidTxGPIOpin(int8_t pin) {
    return isValidGPIOpin(pin)
#if defined(ESP32)
#ifdef CONFIG_IDF_TARGET_ESP32
        && (pin < 34)
#elif CONFIG_IDF_TARGET_ESP32S2
        && (pin <= 45)
#elif CONFIG_IDF_TARGET_ESP32C3
        // no restrictions
#endif
#endif
        ;
}

constexpr bool EspBitBanger::hasRxGPIOPullUp(int8_t pin) {
#if defined(ESP32)
    return !(pin >= 34 && pin <= 39);
#else
    (void)pin;
    return true;
#endif
}

void EspBitBanger::setRxGPIOPinMode() {
    if (m_rxValid) {
        pinMode(m_rxPin, hasRxGPIOPullUp(m_rxPin) && m_rxGPIOPullUpEnabled ? INPUT_PULLUP : INPUT);
    }
}

void EspBitBanger::setTxGPIOPinMode() {
    if (m_txValid) {
        pinMode(m_txPin, m_txGPIOOpenDrain ? OUTPUT_OPEN_DRAIN : OUTPUT);
    }
}

void EspBitBanger::begin(uint32_t baud, int8_t rxPin, int8_t txPin, BitBangerBitOrder bitOrder, bool invert, int bufCapacity, int isrBufCapacity) {
    if (-1 != rxPin) m_rxPin = rxPin;
    if (-1 != txPin) m_txPin = txPin;
    m_bitOrder = bitOrder;
    m_invert = invert;
    m_oneWire = (m_rxPin == m_txPin);
    m_pduBits = m_dataBits;
    m_bitTicks = (microsToTicks(1000000UL) + baud / 2) / baud;
    m_intTxEnabled = true;
    if (isValidRxGPIOpin(m_rxPin)) {
        m_rxReg = portInputRegister(digitalPinToPort(m_rxPin));
        m_rxBitMask = digitalPinToBitMask(m_rxPin);
        m_buffer.reset(new circular_queue<uint8_t>((bufCapacity > 0) ? bufCapacity : 64));
        m_isrBuffer.reset(new circular_queue<uint32_t, EspBitBanger*>((isrBufCapacity > 0) ? isrBufCapacity : m_buffer->capacity() * m_dataBits));
        if (m_buffer && m_isrBuffer) {
            m_rxValid = true;
            setRxGPIOPinMode();
        }
    }
    if (isValidTxGPIOpin(m_txPin)) {
#if !defined(ESP8266)
        m_txReg = portOutputRegister(digitalPinToPort(m_txPin));
#endif
        m_txBitMask = digitalPinToBitMask(m_txPin);
        m_txValid = true;
        if (!m_oneWire) {
            setTxGPIOPinMode();
            digitalWrite(m_txPin, 0);
        }
    }
    enableRx(true);
}

void EspBitBanger::end()
{
    enableRx(false);
    m_txValid = false;
    if (m_buffer) {
        m_buffer.reset();
    }
    if (m_isrBuffer) {
        m_isrBuffer.reset();
    }
}

uint32_t EspBitBanger::baudRate() {
    return 1000000UL / ticksToMicros(m_bitTicks);
}

void EspBitBanger::setTransmitEnablePin(int8_t txEnablePin) {
    if (isValidTxGPIOpin(txEnablePin)) {
        m_txEnableValid = true;
        m_txEnablePin = txEnablePin;
        pinMode(m_txEnablePin, OUTPUT);
        digitalWrite(m_txEnablePin, LOW);
    }
    else {
        m_txEnableValid = false;
    }
}

void EspBitBanger::enableIntTx(bool on) {
    m_intTxEnabled = on;
}

void EspBitBanger::enableRxGPIOPullUp(bool on) {
    m_rxGPIOPullUpEnabled = on;
    setRxGPIOPinMode();
}

void EspBitBanger::enableTxGPIOOpenDrain(bool on) {
    m_txGPIOOpenDrain = on;
    setTxGPIOPinMode();
}

void EspBitBanger::enableTx(bool on) {
    if (m_txValid && m_oneWire) {
        if (on) {
            enableRx(false);
            setTxGPIOPinMode();
            digitalWrite(m_txPin, 0);
        }
        else {
            setRxGPIOPinMode();
            enableRx(true);
        }
    }
}

void EspBitBanger::enableRx(bool on) {
    if (m_rxValid && on != m_rxEnabled) {
        if (on) {
            m_rxLastBit = m_pduBits - 1;
            // Init to stop bit level and current tick
            m_isrLastTick = (microsToTicks(micros()) | 1);
            if (m_bitTicks >= microsToTicks(1000000UL / 74880UL))
                attachInterruptArg(digitalPinToInterrupt(m_rxPin), reinterpret_cast<void (*)(void*)>(rxBitISR), this, CHANGE);
            else
                attachInterruptArg(digitalPinToInterrupt(m_rxPin), reinterpret_cast<void (*)(void*)>(rxBitSyncISR), this, FALLING);
        }
        else {
            detachInterrupt(digitalPinToInterrupt(m_rxPin));
        }
        m_rxEnabled = on;
    }
}

int EspBitBanger::read() {
    if (!m_rxValid) { return -1; }
    if (!m_buffer->available()) {
        rxBits();
        if (!m_buffer->available()) { return -1; }
    }
    auto val = m_buffer->pop();
    return val;
}

int EspBitBanger::read(uint8_t* buffer, size_t size) {
    if (!m_rxValid) { return 0; }
    int avail;
    if (0 == (avail = m_buffer->pop_n(buffer, size))) {
        rxBits();
        avail = m_buffer->pop_n(buffer, size);
    }
    if (!avail) return 0;
    return avail;
}

size_t EspBitBanger::readBytes(uint8_t* buffer, size_t size) {
    if (!m_rxValid || !size) { return 0; }
    size_t count = 0;
    auto start = millis();
    do {
        auto readCnt = read(&buffer[count], size - count);
        count += readCnt;
        if (count >= size) break;
        if (readCnt) {
            start = millis();
        }
        else {
            optimistic_yield(1000UL);
        }
    } while (millis() - start < _timeout);
    return count;
}

int EspBitBanger::available() {
    if (!m_rxValid) { return 0; }
    rxBits();
    int avail = m_buffer->available();
    if (!avail) {
        optimistic_yield(10000UL);
    }
    return avail;
}

void EspBitBanger::lazyDelay() {
    // Reenable interrupts while delaying to avoid other tasks piling up
    if (!m_intTxEnabled) { restoreInterrupts(); }
    const auto expired = microsToTicks(micros()) - m_periodStart;
    const int32_t remaining = m_periodDuration - expired;
    const uint32_t ms = remaining > 0 ? ticksToMicros(remaining) / 1000UL : 0;
    if (ms > 0)
    {
        delay(ms);
    }
    else
    {
        optimistic_yield(10000UL);
    }
    // Assure that below-ms part of delays are not elided
    preciseDelay();
    // Disable interrupts again if applicable
    if (!m_intTxEnabled) { disableInterrupts(); }
}

void IRAM_ATTR EspBitBanger::preciseDelay() {
    uint32_t ticks;
    do {
        ticks = microsToTicks(micros());
    } while ((ticks - m_periodStart) < m_periodDuration);
    m_periodDuration = 0;
    m_periodStart = ticks;
}

void IRAM_ATTR EspBitBanger::writeBit(bool bit, uint32_t duration) {
    if (bit) {
#if defined(ESP8266)
        if (16 == m_txPin) {
            GP16O = 1;
        }
        else {
            GPOS = m_txBitMask;
        }
#else
        *m_txReg |= m_txBitMask;
#endif
    } else {
#if defined(ESP8266)
        if (16 == m_txPin) {
            GP16O = 0;
        }
        else {
            GPOC = m_txBitMask;
        }
#else
        *m_txReg &= ~m_txBitMask;
#endif
        
    }
    m_periodDuration += duration;
    preciseDelay();
}

size_t EspBitBanger::write(uint8_t byte) {
    return write(&byte, 1);
}

size_t IRAM_ATTR EspBitBanger::write(const uint8_t* buffer, size_t size) {
    if (m_rxValid) { rxBits(); }
    if (!m_txValid) {return -1;}
    
    uint32_t duration = 0;
    if (!m_intTxEnabled) {
        // Disable interrupts in order to get a clean transmit timing
        disableInterrupts();
    }
    m_periodDuration = 0;
    m_periodStart = microsToTicks(micros());
    bool b = digitalRead(m_txPin);
    bool pb;
    for (size_t cnt = 0; cnt < size; ++cnt) {
        uint8_t byte = pgm_read_byte(buffer + cnt);
        if(m_invert){
            byte = ~byte;
        }
        for (int i = 0; i < m_pduBits; ++i) {
            pb = b;
            if(m_bitOrder == MSB_FIRST){
                b = (byte << i) & 0b10000000;
            } else {
                b = (byte >> i) & 0b00000001;
            }
            if (b != pb) {
                writeBit(pb, duration);
                duration = 0;
            }
            duration += m_bitTicks;
        }
    }
    writeBit(b, duration);
    duration = 0;
    if (!m_intTxEnabled) {
        // restore the interrupt state if applicable
        restoreInterrupts();
    }
    if (m_txEnableValid) {
        digitalWrite(m_txEnablePin, LOW);
    }
    return size;
}

void EspBitBanger::flush() {
    if (!m_rxValid) { return; }
    m_buffer->flush();
}

bool EspBitBanger::overflow() {
    bool res = m_overflow;
    m_overflow = false;
    return res;
}

int EspBitBanger::peek() {
    if (!m_rxValid) { return -1; }
    if (!m_buffer->available()) {
        rxBits();
        if (!m_buffer->available()) return -1;
    }
    auto val = m_buffer->peek();
    return val;
}

void EspBitBanger::rxBits() {
#ifdef ESP8266
    if (m_isrOverflow.load()) {
        m_overflow = true;
        m_isrOverflow.store(false);
    }
#else
    if (m_isrOverflow.exchange(false)) {
        m_overflow = true;
    }
#endif

    m_isrBuffer->for_each(m_isrBufferForEachDel);
}

void EspBitBanger::rxBits(const uint32_t isrTick) {
    const bool level = (m_isrLastTick & 1);

    // error introduced by edge value in LSB of isrTick is negligible
    uint32_t ticks = isrTick - m_isrLastTick;
    m_isrLastTick = isrTick;

    uint32_t bits = ticks / m_bitTicks;
    if (ticks % m_bitTicks > (m_bitTicks >> 1)) ++bits;
    while (bits > 0) {
        // start bit detection
        if (m_rxLastBit >= (m_pduBits - 1)) {
            m_rxLastBit = -1;
        }
        // data bits
        if (m_rxLastBit < (m_dataBits - 1)) {
            uint8_t dataBits = min(bits, static_cast<uint32_t>(m_dataBits - 1 - m_rxLastBit));
            m_rxLastBit += dataBits;
            bits -= dataBits;
            m_rxCurByte >>= dataBits;
            if (level){
                m_rxCurByte |= (BYTE_ALL_BITS_SET << (8 - dataBits));
            }
            continue;
        }
        // stop bits
        // Store the received value in the buffer unless we have an overflow
        if (bits >= static_cast<uint32_t>(m_pduBits - 1 - m_rxLastBit)) {
            m_rxCurByte >>= (sizeof(uint8_t) * 8 - m_dataBits);
            if (!m_buffer->push(m_rxCurByte)) {
                m_overflow = true;
            }
        }
        m_rxLastBit = m_pduBits - 1;
        // reset to 0 is important for masked bit logic
        m_rxCurByte = 0;
        break;
    }
}

void IRAM_ATTR EspBitBanger::rxBitISR(EspBitBanger* self) {
    const bool level = *self->m_rxReg & self->m_rxBitMask;
    const uint32_t curTick = microsToTicks(micros());
    const bool empty = !self->m_isrBuffer->available();

    // Store level and tick in the buffer unless we have an overflow
    // tick's LSB is repurposed for the level bit
    if (!self->m_isrBuffer->push((curTick | 1U) ^ !level)) self->m_isrOverflow.store(true);
    // Trigger rx callback only when receiver is starved
    if (empty && self->m_rxHandler) self->m_rxHandler();
}

void IRAM_ATTR EspBitBanger::rxBitSyncISR(EspBitBanger* self) {
    bool level = 0;
    const uint32_t start = microsToTicks(micros());
    uint32_t wait = self->m_bitTicks - microsToTicks(2U);
    const bool empty = !self->m_isrBuffer->available();

    // Store level and tick in the buffer unless we have an overflow
    // tick's LSB is repurposed for the level bit
    if (!self->m_isrBuffer->push(((start + wait) | 1U) ^ !level)) self->m_isrOverflow.store(true);

    for (uint32_t i = 0; i < self->m_pduBits; ++i) {
        while (microsToTicks(micros()) - start < wait) {};
        wait += self->m_bitTicks;

        // Store level and tick in the buffer unless we have an overflow
        // tick's LSB is repurposed for the level bit
        if (static_cast<bool>(*self->m_rxReg & self->m_rxBitMask) != level)
        {
            if (!self->m_isrBuffer->push(((start + wait) | 1U) ^ level)) self->m_isrOverflow.store(true);
            level = !level;
        }
    }
    // Trigger rx callback only when receiver is starved
    if (empty && self->m_rxHandler) self->m_rxHandler();
}

void EspBitBanger::onReceive(Delegate<void(), void*> handler) {
    m_rxHandler = handler;
}

