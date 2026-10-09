#pragma once

// SPI / I2C driver for the ST H3LIS331DL high-g accelerometer (also the
// LIS331HH/DLH, which share the register map at lower ranges). Written
// against the ST datasheet register descriptions.
//
// Output in g in the chip's own axes. Default configuration (see
// configure()): 400 Hz, +/-400 g, normal power mode, all axes on.

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

class AlfredoH3LIS {
  public:
    // SPI, 4-wire. The chip answers in SPI mode 0 or 3; mode 0 is what the
    // ESP32 SPI library defaults to and what the Rotini boards run.
    bool begin(SPIClass& spi, uint8_t csPin) {
        _spi = &spi;
        _cs = csPin;
        _wire = nullptr;
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        return begin();
    }

    // I2C, address 0x18 (SA0 low) or 0x19 (SA0 high).
    bool begin(TwoWire& wire, uint8_t address = 0x18) {
        _wire = &wire;
        _address = address;
        _spi = nullptr;
        return begin();
    }

    // Output data rate in Hz (50, 100, 400, 1000) and full scale in g (100,
    // 200, 400). Values are rounded down to the nearest supported setting.
    bool configure(uint16_t odrHz = 400, uint16_t rangeG = 400) {
        uint8_t dr = 0;                      // 00: 50 Hz
        if (odrHz >= 1000) dr = 3;           // 11: 1000 Hz
        else if (odrHz >= 400) dr = 2;       // 10: 400 Hz
        else if (odrHz >= 100) dr = 1;       // 01: 100 Hz
        uint8_t fs = 0; _mgPerLsb = 49;      // 00: +/-100 g
        if (rangeG >= 400) { fs = 3; _mgPerLsb = 195; }       // 11: +/-400 g
        else if (rangeG >= 200) { fs = 1; _mgPerLsb = 98; }   // 01: +/-200 g

        bool ok = writeRegister(REG_CTRL1, 0x20 | (dr << 3) | 0x07);  // normal mode, ODR, XYZ on
        ok &= writeRegister(REG_CTRL2, 0x00);                        // high-pass filter off
        ok &= writeRegister(REG_CTRL4, 0x80 | (fs << 4));            // block data update, full scale
        return ok;
    }

    bool available() {  // a new XYZ sample is ready
        int status = readRegister(REG_STATUS);
        return status >= 0 && (status & 0x08);
    }

    // Acceleration in g. Reading clears data-ready (and the INT1 line).
    bool read(float& x, float& y, float& z) {
        int16_t rx, ry, rz;
        if (!readRaw(rx, ry, rz)) return false;
        const float k = _mgPerLsb * 0.001f;
        x = rx * k; y = ry * k; z = rz * k;
        return true;
    }

    // 12-bit signed counts (-2048..2047).
    bool readRaw(int16_t& x, int16_t& y, int16_t& z) {
        int16_t raw[3];  // 12-bit left-justified in 16
        if (!readRegisters(REG_OUT_X_L, (uint8_t*)raw, sizeof(raw))) return false;
        x = raw[0] >> 4; y = raw[1] >> 4; z = raw[2] >> 4;
        return true;
    }

    // Data-ready on the INT1 pin, active high, push-pull.
    bool enableInterrupt() { return writeRegister(REG_CTRL3, 0x02); }

    bool isConnected() { return readRegister(REG_WHO_AM_I) == 0x32; }

  private:
    enum : uint8_t {
        REG_WHO_AM_I = 0x0F,
        REG_CTRL1    = 0x20,
        REG_CTRL2    = 0x21,
        REG_CTRL3    = 0x22,
        REG_CTRL4    = 0x23,
        REG_STATUS   = 0x27,
        REG_OUT_X_L  = 0x28,
    };

    bool begin() {
        if (!isConnected()) return false;
        return configure();
    }

    int readRegister(uint8_t reg) {  // value, or -1 on failure
        uint8_t value;
        return readRegisters(reg, &value, 1) ? value : -1;
    }

    // Bit 7 = read, bit 6 = address auto-increment (both buses).
    bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
        if (_spi) {
            _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
            digitalWrite(_cs, LOW);
            _spi->transfer(reg | 0xC0);
            while (length-- > 0) *data++ = _spi->transfer(0);
            digitalWrite(_cs, HIGH);
            _spi->endTransaction();
            return true;
        }
        if (!_wire) return false;
        _wire->beginTransmission(_address);
        _wire->write(reg | 0x80);
        if (_wire->endTransmission(false) != 0) return false;
        if (_wire->requestFrom(_address, (uint8_t)length) != length) return false;
        while (length-- > 0) *data++ = _wire->read();
        return true;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        if (_spi) {
            _spi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
            digitalWrite(_cs, LOW);
            _spi->transfer(reg);
            _spi->transfer(value);
            digitalWrite(_cs, HIGH);
            _spi->endTransaction();
            return true;
        }
        if (!_wire) return false;
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->write(value);
        return _wire->endTransmission() == 0;
    }

    SPIClass* _spi = nullptr;
    uint8_t _cs = 0;
    TwoWire* _wire = nullptr;
    uint8_t _address = 0x18;
    float _mgPerLsb = 195;  // sensitivity at the configured full scale
};
