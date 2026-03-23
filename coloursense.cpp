#include "colorsensor.hpp"

namespace tcs3472 {

// 7-bit I2C address for TCS3472x family
static constexpr uint8_t ADDR_7BIT = 0x29;

// Command bit + auto-increment
static constexpr uint8_t CMD_BIT     = 0x80;
static constexpr uint8_t CMD_AUTOINC = 0x20;

// Registers
static constexpr uint8_t REG_ENABLE  = 0x00;
static constexpr uint8_t REG_ATIME   = 0x01;
static constexpr uint8_t REG_CONTROL = 0x0F;
static constexpr uint8_t REG_ID      = 0x12; // optional
static constexpr uint8_t REG_STATUS  = 0x13;

static constexpr uint8_t REG_CDATAL  = 0x14; // C low/high then R, G, B

// ENABLE bits
static constexpr uint8_t ENABLE_PON  = 0x01; // Power ON
static constexpr uint8_t ENABLE_AEN  = 0x02; // ADC Enable

// STATUS bits
static constexpr uint8_t STATUS_AVALID = 0x01;

TCS3472::TCS3472(PinName sda, PinName scl, int hz)
    : _i2c(sda, scl)
{
    _i2c.frequency(hz);
}

uint8_t TCS3472::atime_from_ms(float ms) {
    // integration_ms = 2.4ms * (256 - ATIME)
    if (ms < 2.4f)   ms = 2.4f;
    if (ms > 614.4f) ms = 614.4f;

    float cycles = ms / 2.4f;              // (256 - ATIME)
    int atime = 256 - (int)(cycles + 0.5f);
    if (atime < 0)   atime = 0;
    if (atime > 255) atime = 255;
    return (uint8_t)atime;
}

float TCS3472::gain_multiplier(Gain g) {
    switch (g) {
        case Gain::X1:  return 1.0f;
        case Gain::X4:  return 4.0f;
        case Gain::X16: return 16.0f;
        case Gain::X60: return 60.0f;
        default:        return 1.0f;
    }
}

bool TCS3472::init(float integration_ms, Gain gain) {
    // Optional ID read; don't hard-fail if it returns something unexpected.
    uint8_t dummy = 0;
    (void)read8(REG_ID, dummy);

    // Set integration time + gain
    if (!write8(REG_ATIME, atime_from_ms(integration_ms))) return false;
    if (!write8(REG_CONTROL, static_cast<uint8_t>(gain)))  return false;

    // Power on
    if (!write8(REG_ENABLE, ENABLE_PON)) return false;
    ThisThread::sleep_for(3ms);

    // Enable RGBC ADC
    if (!write8(REG_ENABLE, ENABLE_PON | ENABLE_AEN)) return false;

    _integration_ms = integration_ms;
    _gain = gain;
    return true;
}

RGBC TCS3472::read_raw(bool wait_for_valid, int timeout_ms) {
    RGBC out{0, 0, 0, 0, false};

    if (wait_for_valid) {
        Timer t;
        t.start();
        while (t.elapsed_time() < chrono::milliseconds(timeout_ms)) {
            uint8_t st = 0;
            if (read8(REG_STATUS, st) && (st & STATUS_AVALID)) {
                out.valid = true;
                break;
            }
            ThisThread::sleep_for(2ms);
        }
    } else {
        uint8_t st = 0;
        if (read8(REG_STATUS, st) && (st & STATUS_AVALID)) out.valid = true;
    }

    // Burst read 8 bytes: C,R,G,B low/high
    uint8_t buf[8] = {0};
    if (!readN(REG_CDATAL, buf, sizeof(buf))) {
        out.valid = false;
        return out;
    }

    out.c = (uint16_t)(buf[0] | (buf[1] << 8));
    out.r = (uint16_t)(buf[2] | (buf[3] << 8));
    out.g = (uint16_t)(buf[4] | (buf[5] << 8));
    out.b = (uint16_t)(buf[6] | (buf[7] << 8));

    return out;
}

float TCS3472::estimate_lux(const RGBC& v) const {
    // Rough clear-channel proxy, compensated for gain + integration time.
    float g = gain_multiplier(_gain);
    float it = (_integration_ms > 0.0f) ? (50.0f / _integration_ms) : 1.0f;
    return (v.c / g) * it;
}

float TCS3472::estimate_cct_kelvin(const RGBC& v) const {
    // Rough CCT proxy based on B/R ratio (very setup-dependent).
    if (v.r == 0) return 0.0f;
    float br = (float)v.b / (float)v.r;
    return 1000.0f + 5000.0f * br;
}

// ---------------- low-level I2C helpers ----------------

bool TCS3472::write8(uint8_t reg, uint8_t val) {
    char data[2];
    data[0] = (char)(CMD_BIT | reg);
    data[1] = (char)val;
    // repeated=false => STOP at end (normal for write)
    return (_i2c.write((int)(ADDR_7BIT << 1), data, 2, false) == 0);
}

bool TCS3472::read8(uint8_t reg, uint8_t &out) {
    char cmd = (char)(CMD_BIT | reg);
    // repeated=true => keep bus for repeated-start
    if (_i2c.write((int)(ADDR_7BIT << 1), &cmd, 1, true) != 0) return false;

    char val = 0;
    // repeated=false => STOP after read
    if (_i2c.read((int)(ADDR_7BIT << 1), &val, 1, false) != 0) return false;

    out = (uint8_t)val;
    return true;
}

bool TCS3472::readN(uint8_t start_reg, uint8_t* buf, size_t n) {
    char cmd = (char)(CMD_BIT | CMD_AUTOINC | start_reg);
    if (_i2c.write((int)(ADDR_7BIT << 1), &cmd, 1, true) != 0) return false;
    return (_i2c.read((int)(ADDR_7BIT << 1), (char*)buf, (int)n, false) == 0);
}

} // namespace tcs3472