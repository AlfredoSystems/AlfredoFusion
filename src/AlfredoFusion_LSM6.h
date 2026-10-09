#pragma once

// I2C driver for the ST LSM6 family of accelerometer/gyroscopes. Written
// against the ST datasheet register descriptions.
//
// Detected by WHO_AM_I: LSM6DS3, LSM6DS3TR-C, LSM6DSL, LSM6DSM, LSM6DSO,
// LSM6DSOX, LSM6DSO32(X), LSM6DSR(X), LSM6DST(X), ISM330DHCX - every part that
// shares the classic register map. The LSM6DSV family has a different
// control register layout and is not supported.
//
// Output is in the chip's own axes: accel in g, gyro in rad/s. Default
// configuration (see configure()): 104 Hz, +/-4 g, +/-500 dps, high
// performance mode, accel low-pass at ODR/4.

#include <Arduino.h>
#include <Wire.h>

class AlfredoLSM6 {
  public:
    // Detects the chip (address 0x6A or 0x6B, or pass one), resets and
    // configures it with the defaults. Returns false if nothing answered.
    bool begin(TwoWire& wire, uint8_t address = 0) {
        _wire = &wire;
        const uint8_t addresses[2] = {0x6B, 0x6A};
        for (int i = 0; i < 2; i++) {
            if (address != 0 && address != addresses[i]) continue;
            _address = addresses[i];
            int id = readRegister(REG_WHO_AM_I);
            if (id == 0x69 || id == 0x6A || id == 0x6B || id == 0x6C || id == 0x6D) {
                _whoAmI = id;
                return configure();
            }
        }
        _address = 0;
        return false;
    }

    // Output data rate in Hz (12.5, 26, 52, 104, 208, 416, 833, 1660, 3330,
    // 6660; both sensors share it), accel full scale in g (2, 4, 8, 16) and
    // gyro full scale in dps (125, 250, 500, 1000, 2000). Values are rounded
    // down to the nearest supported setting.
    bool configure(float odrHz = 104, uint8_t accelRangeG = 4, uint16_t gyroRangeDps = 500) {
        // Software reset: the chip keeps stale state through a host warm reset.
        writeRegister(REG_CTRL3_C, 0x01);
        unsigned long start = millis();
        while ((readRegister(REG_CTRL3_C) & 0x01) && millis() - start < 100) delay(1);

        static const float odrTable[] = {12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660};
        uint8_t odrBits = 1;
        for (uint8_t i = 0; i < 10; i++) if (odrHz >= odrTable[i]) odrBits = i + 1;
        _odrHz = odrTable[odrBits - 1];

        uint8_t fsXl = 0; _accelG = 2;                           // 00: +/-2 g
        if (accelRangeG >= 16) { fsXl = 1; _accelG = 16; }       // 01: +/-16 g
        else if (accelRangeG >= 8) { fsXl = 3; _accelG = 8; }    // 11: +/-8 g
        else if (accelRangeG >= 4) { fsXl = 2; _accelG = 4; }    // 10: +/-4 g

        uint8_t fsG = 0; _gyroMdps = 8.75f;                                    // 00: +/-250 dps
        if (gyroRangeDps >= 2000) { fsG = 0x0C; _gyroMdps = 70.0f; }           // 11: +/-2000 dps
        else if (gyroRangeDps >= 1000) { fsG = 0x08; _gyroMdps = 35.0f; }      // 10: +/-1000 dps
        else if (gyroRangeDps >= 500) { fsG = 0x04; _gyroMdps = 17.5f; }       // 01: +/-500 dps
        else if (gyroRangeDps < 250) { fsG = 0x02; _gyroMdps = 4.375f; }       // FS_125

        bool ok = true;
        if (_whoAmI >= 0x6B) ok &= writeRegister(REG_CTRL9_XL, 0xE2);  // DSO/DSR/DST: disable I3C
        ok &= writeRegister(REG_CTRL3_C, 0x44);                        // block data update + auto-increment
        ok &= writeRegister(REG_CTRL2_G, (odrBits << 4) | fsG);
        ok &= writeRegister(REG_CTRL1_XL, (odrBits << 4) | (fsXl << 2) | 0x02);  // LPF2 on (DSO) / 100 Hz AA (DS3)
        ok &= writeRegister(REG_CTRL7_G, 0x00);                        // gyro high-performance mode
        ok &= writeRegister(REG_CTRL8_XL, 0x00);                       // accel low-pass at ODR/4
        return ok;
    }

    // True when both sensors have a fresh sample.
    bool available() {
        int status = readRegister(REG_STATUS);
        return status >= 0 && (status & 0x03) == 0x03;
    }

    // Both sensors in one 12-byte burst (the output registers are contiguous)
    // so the two samples are from the same instant. Accel in g, gyro in rad/s.
    bool read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
        int16_t raw[6];  // gyro X,Y,Z then accel X,Y,Z
        if (!readRegisters(REG_OUTX_L_G, (uint8_t*)raw, sizeof(raw))) return false;
        const float kG = _gyroMdps * 0.001f * DEG_TO_RAD;
        const float kA = _accelG / 32768.0f;
        gx = raw[0] * kG; gy = raw[1] * kG; gz = raw[2] * kG;
        ax = raw[3] * kA; ay = raw[4] * kA; az = raw[5] * kA;
        return true;
    }

    bool readAcceleration(float& x, float& y, float& z) {  // g
        int16_t raw[3];
        if (!readRegisters(REG_OUTX_L_A, (uint8_t*)raw, sizeof(raw))) return false;
        const float k = _accelG / 32768.0f;
        x = raw[0] * k; y = raw[1] * k; z = raw[2] * k;
        return true;
    }

    bool readGyroscope(float& x, float& y, float& z) {  // rad/s
        int16_t raw[3];
        if (!readRegisters(REG_OUTX_L_G, (uint8_t*)raw, sizeof(raw))) return false;
        const float k = _gyroMdps * 0.001f * DEG_TO_RAD;
        x = raw[0] * k; y = raw[1] * k; z = raw[2] * k;
        return true;
    }

    bool readTemperature(float& degC) {
        int16_t raw;
        if (!readRegisters(REG_OUT_TEMP_L, (uint8_t*)&raw, sizeof(raw))) return false;
        degC = raw / 256.0f + 25.0f;  // 256 LSB per degree C, 0 LSB = 25 C
        return true;
    }

    // Route accel and gyro data-ready to the INT1 pin (active high). The
    // line drops when the data is read.
    bool enableInterrupt() { return writeRegister(REG_INT1_CTRL, 0x03); }

    float sampleRate() const { return _odrHz; }  // Hz, as configured
    uint8_t whoAmI() const { return _whoAmI; }
    const char* name() const {
        switch (_whoAmI) {
            case 0x69: return "LSM6DS3";
            case 0x6A: return "LSM6DS3TR-C/DSL/DSM";
            case 0x6B: return "LSM6DSR/ISM330DHCX";
            case 0x6C: return "LSM6DSO/DSOX/DSO32";
            case 0x6D: return "LSM6DST";
            default: return "none";
        }
    }

  private:
    enum : uint8_t {
        REG_INT1_CTRL  = 0x0D,
        REG_WHO_AM_I   = 0x0F,
        REG_CTRL1_XL   = 0x10,
        REG_CTRL2_G    = 0x11,
        REG_CTRL3_C    = 0x12,
        REG_CTRL7_G    = 0x16,
        REG_CTRL8_XL   = 0x17,
        REG_CTRL9_XL   = 0x18,
        REG_STATUS     = 0x1E,
        REG_OUT_TEMP_L = 0x20,
        REG_OUTX_L_G   = 0x22,
        REG_OUTX_L_A   = 0x28,
    };

    int readRegister(uint8_t reg) {  // value, or -1 on failure
        uint8_t value;
        return readRegisters(reg, &value, 1) ? value : -1;
    }

    bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
        if (_address == 0) return false;
        _wire->beginTransmission(_address);
        _wire->write(reg);
        if (_wire->endTransmission(false) != 0) return false;
        if (_wire->requestFrom(_address, (uint8_t)length) != length) return false;
        while (length-- > 0) *data++ = _wire->read();
        return true;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        if (_address == 0) return false;
        _wire->beginTransmission(_address);
        _wire->write(reg);
        _wire->write(value);
        return _wire->endTransmission() == 0;
    }

    TwoWire* _wire = nullptr;
    uint8_t _address = 0;  // 0 until a chip has been detected
    uint8_t _whoAmI = 0;
    float _odrHz = 104;
    float _accelG = 4;
    float _gyroMdps = 17.5f;  // sensitivity, mdps per LSB
};
