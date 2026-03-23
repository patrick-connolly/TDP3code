#include "mbed.h"
#include "colorsensor.hpp"
#include <cstdio>
#include <cstring>
#include <cstdarg>

using namespace tcs3472;

// USB serial on KL25Z
BufferedSerial pc(USBTX, USBRX, 115200);

static void println(const char* s) {
    pc.write(s, strlen(s));
    pc.write("\r\n", 2);
}

static void print_fmt(const char* fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    pc.write(buf, strlen(buf));
}

// Super simple classifier using normalized RGB.
// You WILL tune these thresholds later with real data.
static const char* classify_traffic(float rn, float gn, float bn) {
    // Basic sanity: if green and red both high, treat as amber/yellow-ish.
    if (rn > 0.45f && gn > 0.35f && bn < 0.25f) return "AMBER";
    if (rn > gn + 0.12f && rn > bn + 0.12f)     return "RED";
    if (gn > rn + 0.12f && gn > bn + 0.12f)     return "GREEN";
    return "UNKNOWN";
}

int main() {
    println("--- KL25Z + TCS3472 (M5 Color Unit) ---");
    println("Wiring: SDA=PTE0(D14), SCL=PTE1(D15), VCC=3.3V, GND");

    // Create sensor on KL25Z I2C pins
    TCS3472 sensor(PTE0, PTE1, 100000); // insert specific pins here (SDA, SCL)

    // For bright traffic lights: start with short-ish integration and low/moderate gain.
    // If values are tiny: increase gain or integration_ms.
    // If values saturate (~65535): reduce gain or integration_ms.
    if (!sensor.init(50.0f, Gain::X4)) {
        println("ERROR: sensor.init failed. Check wiring and 3.3V power.");
        while (true) ThisThread::sleep_for(500ms);
    }

    println("OK. Printing: C R G B | rn gn bn | guess");

    while (true) {
        // timeout a bit > integration time is fine
        RGBC v = sensor.read_raw(true, 250);

        if (!v.valid || v.c == 0) {
            println("No valid data yet...");
            ThisThread::sleep_for(100ms);
            continue;
        }

        // Normalize to clear channel for brightness independence
        float rn = (float)v.r / (float)v.c;
        float gn = (float)v.g / (float)v.c;
        float bn = (float)v.b / (float)v.c;

        const char* state = classify_traffic(rn, gn, bn);

        print_fmt("C=%5u R=%5u G=%5u B=%5u | rn=%.3f gn=%.3f bn=%.3f | %s\r\n",
                  v.c, v.r, v.g, v.b, rn, gn, bn, state);

        ThisThread::sleep_for(200ms);
    }
}