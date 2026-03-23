#pragma once
#include "mbed.h"
#include <cstdint>

namespace tcs3472 {

enum class Gain : uint8_t {
    X1  = 0x00,
    X4  = 0x01,
    X16 = 0x02,
    X60 = 0x03
};

struct RGBC {
    uint16_t c;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    bool     valid;
};

class TCS3472 {
public:
    TCS3472(PinName sda, PinName scl, int hz = 100000);

    // Configure sensor. Typical: integration_ms=24..100, gain=X1/X4 for bright sources.
    bool init(float integration_ms = 50.0f, Gain gain = Gain::X4);

    // Read raw RGBC. If wait_for_valid, waits for STATUS.AVALID before reading.
    RGBC read_raw(bool wait_for_valid = true, int timeout_ms = 200);

    // Optional helpers (rough, not calibrated lux/CCT)
    float estimate_lux(const RGBC& v) const;
    float estimate_cct_kelvin(const RGBC& v) const;

private:
    I2C _i2c;
    float _integration_ms = 50.0f;
    Gain  _gain = Gain::X4;

    static uint8_t atime_from_ms(float ms);
    static float gain_multiplier(Gain g);

    bool write8(uint8_t reg, uint8_t val);
    bool read8(uint8_t reg, uint8_t &out);
    bool readN(uint8_t start_reg, uint8_t* buf, size_t n);
};

} // namespace tcs3472