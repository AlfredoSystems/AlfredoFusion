#pragma once

// SPI / I2C driver for the MEMSIC MMC5983MA magnetometer. Written against
// the MEMSIC datasheet register descriptions.
//
// Output in microteslas (+/-800 uT full scale, 18-bit) in the chip's own
// axes. Default configuration (see configure()): continuous mode at 100 Hz,
// 800 Hz filter bandwidth, automatic set/reset on.
//
// The chip's control registers are write-only, so the driver keeps a local
// copy of each and always writes through it, preserving previously set bits.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

class AlfredoMMC5983 {
  public:
    // I2C, address 0x30.
    bool begin(TwoWire& wire) {
        _wire = &wire;
        _spi = nullptr;
        return begin();
    }

    // SPI, 4-wire. Mode 0 is what works in practice on the Rotini boards
    // (the datasheet says mode 3; the chip accepts both).
    bool begin(SPIClass& spi, uint8_t csPin) {
        _spi = &spi;
        _cs = csPin;
        _wire = nullptr;
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        return begin();
    }

    // Continuous measurement rate in Hz (1, 10, 20, 50, 100, 200, 1000; 0 =
    // off, single measurements only), filter bandwidth in Hz (100, 200, 400,
    // 800 - a rate of 200 needs >= 200, 1000 needs 800) and automatic
    // set/reset, which cancels the sensor bridge's offset drift at the cost
    // of half the output rate. Values are rounded down to supported settings.
    bool configure(uint16_t rateHz = 100, uint16_t bandwidthHz = 800, bool autoSetReset = true) {
        uint8_t bw = 0;                          // 100 Hz
        if (bandwidthHz >= 800) bw = 3;
        else if (bandwidthHz >= 400) bw = 2;
        else if (bandwidthHz >= 200) bw = 1;
        static const uint16_t rates[] = {1, 10, 20, 50, 100, 200, 1000};
        uint8_t cm = 0;
        for (uint8_t i = 0; i < 7; i++) if (rateHz >= rates[i]) cm = i + 1;

        _control0 = autoSetReset ? 0x20 : 0x00;
        _control1 = bw;
        _control2 = cm ? (0x08 | cm) : 0x00;  // continuous mode enable + rate
        bool ok = writeRegister(REG_CONTROL_0, _control0);
        ok &= writeRegister(REG_CONTROL_1, _control1);
        ok &= writeRegister(REG_CONTROL_2, _control2);
        return ok;
    }

    bool isConnected() { return readRegister(REG_PRODUCT_ID) == 0x30; }

    // Software reset; takes ~10 ms and zeroes all control registers.
    bool softReset() {
        bool ok = writeRegister(REG_CONTROL_1, 0x80);
        _control0 = _control1 = _control2 = 0;
        delay(15);
        return ok;
    }

    // A measurement has completed since the flag was last cleared.
    bool available() {
        int status = readRegister(REG_STATUS);
        return status >= 0 && (status & 0x01);
    }

    // Field in microteslas. Clears the measurement-done flag (releasing the
    // INT pin) first, so a sample that lands during the read is not lost.
    bool read(float& x, float& y, float& z) {
        uint32_t rx, ry, rz;
        if (!readRaw(rx, ry, rz)) return false;
        const float k = 800.0f / 131072.0f;  // uT per count
        x = ((float)rx - 131072.0f) * k;
        y = ((float)ry - 131072.0f) * k;
        z = ((float)rz - 131072.0f) * k;
        return true;
    }

    // Raw 18-bit counts; 131072 = zero field.
    bool readRaw(uint32_t& x, uint32_t& y, uint32_t& z) {
        clearInterrupt();
        uint8_t b[7];
        if (!readRegisters(REG_XOUT_0, b, sizeof(b))) return false;
        // Each axis: two full bytes, plus its lowest 2 bits packed into b[6].
        x = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | (b[6] >> 6);
        y = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x03);
        z = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x03);
        return true;
    }

    // Single measurement when continuous mode is off (rateHz = 0). Blocks
    // for the measurement (~1-8 ms depending on bandwidth).
    bool readSingle(float& x, float& y, float& z) {
        if (!writeRegister(REG_CONTROL_0, _control0 | 0x01)) return false;  // TM_M
        unsigned long start = millis();
        while (!available() && millis() - start < 20) delay(1);
        return read(x, y, z);
    }

    // Die temperature in degrees C (-75..125). Blocks for the measurement.
    bool readTemperature(float& degC) {
        if (!writeRegister(REG_CONTROL_0, _control0 | 0x02)) return false;  // TM_T
        unsigned long start = millis();
        while (!(readRegister(REG_STATUS) & 0x02) && millis() - start < 20) delay(1);
        int raw = readRegister(REG_TOUT);
        if (raw < 0) return false;
        writeRegister(REG_STATUS, 0x02);  // clear temperature-done
        degC = -75.0f + raw * (200.0f / 255.0f);
        return true;
    }

    // Assert the INT pin when a measurement completes; read() re-arms it.
    bool enableInterrupt() {
        _control0 |= 0x04;
        return writeRegister(REG_CONTROL_0, _control0);
    }
    bool disableInterrupt() {
        _control0 &= ~0x04;
        return writeRegister(REG_CONTROL_0, _control0);
    }

    // Clear the measurement-done flag (and release the INT pin) by writing
    // a 1 to it in the status register.
    bool clearInterrupt() { return writeRegister(REG_STATUS, 0x01); }

    // Manual SET / RESET pulses, for use with automatic set/reset off.
    bool performSet() { bool ok = writeRegister(REG_CONTROL_0, _control0 | 0x08); delay(1); return ok; }
    bool performReset() { bool ok = writeRegister(REG_CONTROL_0, _control0 | 0x10); delay(1); return ok; }

  private:
    enum : uint8_t {
        REG_XOUT_0     = 0x00,
        REG_TOUT       = 0x07,
        REG_STATUS     = 0x08,
        REG_CONTROL_0  = 0x09,
        REG_CONTROL_1  = 0x0A,
        REG_CONTROL_2  = 0x0B,
        REG_PRODUCT_ID = 0x2F,
    };
    static const uint8_t I2C_ADDRESS = 0x30;

    bool begin() {
        if (!isConnected()) return false;
        softReset();
        return configure();
    }

    int readRegister(uint8_t reg) {  // value, or -1 on failure
        uint8_t value;
        return readRegisters(reg, &value, 1) ? value : -1;
    }

    bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
        if (_spi) {
            _spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
            digitalWrite(_cs, LOW);
            _spi->transfer(reg | 0x80);  // bit 7 = read
            while (length-- > 0) *data++ = _spi->transfer(0);
            digitalWrite(_cs, HIGH);
            _spi->endTransaction();
            return true;
        }
        if (!_wire) return false;
        _wire->beginTransmission(I2C_ADDRESS);
        _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;
        if (_wire->requestFrom(I2C_ADDRESS, (uint8_t)length) != length) return false;
        while (length-- > 0) *data++ = _wire->read();
        return true;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        if (_spi) {
            _spi->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
            digitalWrite(_cs, LOW);
            _spi->transfer(reg);
            _spi->transfer(value);
            digitalWrite(_cs, HIGH);
            _spi->endTransaction();
            return true;
        }
        if (!_wire) return false;
        _wire->beginTransmission(I2C_ADDRESS);
        _wire->write(reg);
        _wire->write(value);
        return _wire->endTransmission() == 0;
    }

    SPIClass* _spi = nullptr;
    uint8_t _cs = 0;
    TwoWire* _wire = nullptr;
    // Local copies of the write-only control registers
    uint8_t _control0 = 0, _control1 = 0, _control2 = 0;
};
