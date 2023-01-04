/*

EspBitBanger.h - Implementation of a bit banger for proprietary serial protocols.
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

#ifndef __EspBitBanger_h
#define __EspBitBanger_h

#include "circular_queue/circular_queue.h"
#include <Stream.h>
#include <PrintDebug.h>

enum BitBangerBitOrder : uint8_t {
    MSB_FIRST = 0,
    LSB_FIRST = 7,
};

/// This class is compatible with the corresponding AVR one, however,
/// the constructor takes no arguments, for compatibility with the
/// HardwareSerial class.
/// Instead, the begin() function handles pin assignments and logic inversion.
/// It also has optional input buffer capacity arguments for byte buffer and ISR bit buffer.
/// Bitrates up to at least 115200 can be used.
class EspBitBanger : public Stream {
public:
    EspBitBanger();
    /// Ctor to set defaults for pins.
    /// @param rxPin the GPIO pin used for RX
    /// @param txPin -1 for onewire protocol, GPIO pin used for twowire TX
    EspBitBanger(int8_t rxPin, int8_t txPin = -1);
    EspBitBanger(const EspBitBanger&) = delete;
    EspBitBanger& operator= (const EspBitBanger&) = delete;
    virtual ~EspBitBanger();
    /// Configure the EspBitBanger object for use.
    /// @param baud the TX/RX bitrate
    /// @param rxPin -1 or default: either no RX pin, or keeps the rxPin set in the ctor
    /// @param txPin -1 or default: either no TX pin (onewire), or keeps the txPin set in the ctor
    /// @param bufCapacity the capacity for the received bytes buffer
    /// @param isrBufCapacity 0: derived from bufCapacity. The capacity of the internal asynchronous
    ///        bit receive buffer.
    void begin(uint32_t baud, int8_t rxPin, int8_t txPin, BitBangerBitOrder bitOrder = MSB_FIRST, bool invert = false, int bufCapacity = 64, int isrBufCapacity = 0);
    void begin(uint32_t baud, int8_t rxPin) {
        begin(baud, rxPin, m_txPin);
    }
    void begin(uint32_t baud) {
        begin(baud, m_rxPin, m_txPin);
    }

    uint32_t baudRate();
    /// Transmit control pin.
    void setTransmitEnablePin(int8_t txEnablePin);
    /// Enable (default) or disable interrupts during tx.
    void enableIntTx(bool on);
    /// Enable (default) or disable internal rx GPIO pull-up.
    void enableRxGPIOPullUp(bool on);
    /// Enable or disable (default) tx GPIO output mode.
    void enableTxGPIOOpenDrain(bool on);

    bool overflow();

    int available() override;
#if defined(ESP8266)
    int availableForWrite() override {
#else
    int availableForWrite() {
#endif
        if (!m_txValid) return 0;
        return 1;
    }
    int peek() override;
    int read() override;
    /// The read(buffer, size) functions are non-blocking, the same as readBytes but without timeout
    int read(uint8_t* buffer, size_t size)
#if defined(ESP8266)
        override
#endif
        ;
    /// The read(buffer, size) functions are non-blocking, the same as readBytes but without timeout
    int read(char* buffer, size_t size) {
        return read(reinterpret_cast<uint8_t*>(buffer), size);
    }
    /// @returns The number of bytes read into buffer, up to size. Times out if the limit set through
    ///          Stream::setTimeout() is reached.
    size_t readBytes(uint8_t* buffer, size_t size) override;
    /// @returns The number of bytes read into buffer, up to size. Times out if the limit set through
    ///          Stream::setTimeout() is reached.
    size_t readBytes(char* buffer, size_t size) override {
        return readBytes(reinterpret_cast<uint8_t*>(buffer), size);
    }
    void flush() override;
    size_t write(uint8_t byte) override;
    size_t write(const uint8_t* buffer, size_t size) override;
    size_t write(const char* buffer, size_t size) {
        return write(reinterpret_cast<const uint8_t*>(buffer), size);
    }
    operator bool() const {
        return (-1 == m_rxPin || m_rxValid) && (-1 == m_txPin || m_txValid) && !(-1 == m_rxPin && m_oneWire);
    }

    /// Disable or enable interrupts on the rx pin.
    void enableRx(bool on);
    /// One wire control.
    void enableTx(bool on);

    // AVR compatibility methods.
    bool listen() { enableRx(true); return true; }
    void end();
    bool isListening() { return m_rxEnabled; }
    bool stopListening() { enableRx(false); return true; }

    /// onReceive sets a callback that will be called in interrupt context
    /// when data is received.
    /// More precisely, the callback is triggered when EspEspBitBanger detects
    /// a new reception, which may not yet have completed on invocation.
    /// Reading - never from this interrupt context - should therefore be
    /// delayed for the duration of one incoming word.
    void onReceive(Delegate<void(), void*> handler);

    using Print::write;

private:
    // It's legal to exceed the deadline, for instance,
    // by enabling interrupts.
    void lazyDelay();
    // Synchronous precise delay
    void preciseDelay();
    void writeBit(bool bit, uint32_t duration);
    static constexpr bool isValidGPIOpin(int8_t pin);
    static constexpr bool isValidRxGPIOpin(int8_t pin);
    static constexpr bool isValidTxGPIOpin(int8_t pin);
    // result is only defined for a valid Rx GPIO pin
    static constexpr bool hasRxGPIOPullUp(int8_t pin);
    // safely set the pin mode for the Rx GPIO pin
    void setRxGPIOPinMode();
    // safely set the pin mode for the Tx GPIO pin
    void setTxGPIOPinMode();
    /* check m_rxValid that calling is safe */
    void rxBits();
    void rxBits(const uint32_t isrTick);
    static void disableInterrupts();
    static void restoreInterrupts();

    static void rxBitISR(EspBitBanger* self);
    static void rxBitSyncISR(EspBitBanger* self);

    static inline uint32_t microsToTicks(uint32_t micros) {
        return micros << 1;
    }
    static inline uint32_t ticksToMicros(uint32_t ticks) {
        return ticks >> 1;
    }

    // Member variables
    int8_t m_rxPin = -1;
    volatile uint32_t* m_rxReg;
    uint32_t m_rxBitMask;
    int8_t m_txPin = -1;
#if !defined(ESP8266)
    volatile uint32_t* m_txReg;
#endif
    uint32_t m_txBitMask;
    int8_t m_txEnablePin = -1;
    uint8_t m_dataBits = 8;
    bool m_oneWire;
    bool m_rxValid = false;
    bool m_rxEnabled = false;
    bool m_txValid = false;
    bool m_txEnableValid = false;
    bool m_invert;
    /// PDU bits include data, parity and stop bits; the start bit is not counted.
    uint8_t m_pduBits;
    bool m_intTxEnabled;
    bool m_rxGPIOPullUpEnabled;
    bool m_txGPIOOpenDrain;
    bool m_overflow = false;
    BitBangerBitOrder m_bitOrder;
    uint32_t m_bitTicks;
    int8_t m_rxLastBit; // 0 thru (m_pduBits - m_stopBits - 1): data/parity bits. -1: start bit. (m_pduBits - 1): stop bit.
    uint8_t m_rxCurByte = 0;
    std::unique_ptr<circular_queue<uint8_t> > m_buffer;
    uint32_t m_periodStart;
    uint32_t m_periodDuration;
#ifndef ESP32
    static uint32_t m_savedPS;
#else
    static portMUX_TYPE m_interruptsMux;
#endif
    // the ISR stores the relative bit times in the buffer. The inversion corrected level is used as sign bit (2's complement):
    // 1 = positive including 0, 0 = negative.
    std::unique_ptr<circular_queue<uint32_t, EspBitBanger*> > m_isrBuffer;
    const Delegate<void(uint32_t&&), EspBitBanger*> m_isrBufferForEachDel = { [](EspBitBanger* self, uint32_t&& isrTick) { self->rxBits(isrTick); }, this };
    std::atomic<bool> m_isrOverflow;
    uint32_t m_isrLastTick;
    Delegate<void(), void*> m_rxHandler;
};

#endif // __EspBitBanger_h

