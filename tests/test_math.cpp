// Hardware-free unit tests for the pure math helpers.
// Lightweight: no GoogleTest dependency; returns non-zero on first failure.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "ad5933math.h"

namespace
{
int g_failures = 0;

void check(bool cond, const char* expr, const char* file, int line)
{
  if (!cond)
  {
    ++g_failures;
    std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, expr);
  }
}

bool nearly(double a, double b, double tol = 1e-9)
{
  return std::fabs(a - b) <= tol;
}
} // namespace

#define CHECK(cond) check((cond), #cond, __FILE__, __LINE__)

int main()
{
  using namespace ad5933::math;

  // ---- frequencyCode ------------------------------------------------------
  // datasheet formula: code = (f / (refclk/4)) * 2^27
  {
    // 30 kHz with default 16.776 MHz clock: code = (30000 / 4194000) * 2^27 ~= 960427
    uint32_t code = frequencyCode(30000.0, 16776000.0);
    double expected = (30000.0 / (16776000.0 / 4.0)) * (1u << 27);
    CHECK(code == static_cast<uint32_t>(expected));

    // f == 0 -> 0
    CHECK(frequencyCode(0.0, 16776000.0) == 0u);

    // clamping: huge f saturates to 0xFFFFFF
    CHECK(frequencyCode(1e12, 16776000.0) == 0xFFFFFFu);
  }

  // ---- decodeTemperature --------------------------------------------------
  // 25.0 °C -> raw = 25 * 32 = 800 = 0x320 -> msb=0x03 lsb=0x20
  CHECK(nearly(decodeTemperature(0x03, 0x20), 25.0));
  // 0 °C
  CHECK(nearly(decodeTemperature(0x00, 0x00), 0.0));
  // Negative: -1 °C => raw = -32 in 14-bit two's complement = 16384-32 = 16352 = 0x3FE0
  CHECK(nearly(decodeTemperature(0x3F, 0xE0), -1.0));
  // -25 °C => raw = -800 -> 16384-800 = 15584 = 0x3CE0
  CHECK(nearly(decodeTemperature(0x3C, 0xE0), -25.0));

  // ---- decodeImpedance ----------------------------------------------------
  // Positive quadrant: real=3, imag=4 -> mag=5, phase=atan2(4,3)
  {
    auto s = decodeImpedance(0x00, 0x03, 0x00, 0x04);
    CHECK(nearly(s.real, 3.0));
    CHECK(nearly(s.imag, 4.0));
    CHECK(nearly(s.magnitude, 5.0, 1e-9));
    CHECK(nearly(s.phaseDeg, std::atan2(4.0, 3.0) * 180.0 / M_PI, 1e-9));
  }
  // Negative real, negative imag (third quadrant): real=-3, imag=-4
  // -3 = 0xFFFD, -4 = 0xFFFC
  {
    auto s = decodeImpedance(0xFF, 0xFD, 0xFF, 0xFC);
    CHECK(nearly(s.real, -3.0));
    CHECK(nearly(s.imag, -4.0));
    CHECK(nearly(s.magnitude, 5.0, 1e-9));
    CHECK(s.phaseDeg < 0.0); // atan2 in third quadrant yields negative degrees
  }
  // Zero real, positive imag -> phase = +90
  {
    auto s = decodeImpedance(0x00, 0x00, 0x00, 0x01);
    CHECK(nearly(s.phaseDeg, 90.0, 1e-9));
  }

  // ---- gain factor helpers ------------------------------------------------
  // mag=1e-6, R=10k -> gf = 1/(1e-6 * 1e4) = 100
  CHECK(nearly(midpointGainFactor(1e-6, 10000.0), 100.0, 1e-9));

  // slope: gf rises from 1 to 2 over 1000 Hz -> 1e-3
  CHECK(nearly(gainFactorSlope(1.0, 2.0, 1000.0, 2000.0), 1e-3, 1e-12));

  if (g_failures == 0)
  {
    std::puts("All tests passed.");
    return 0;
  }
  std::fprintf(stderr, "%d test(s) failed.\n", g_failures);
  return 1;
}
