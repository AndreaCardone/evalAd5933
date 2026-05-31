#ifndef __AD5933_MATH_H
#define __AD5933_MATH_H

#include <cmath>
#include <cstdint>

#include "ad5933types.h"

namespace ad5933::math
{
  /// Compute the 24-bit DDS frequency code per AD5933 datasheet.
  inline uint32_t frequencyCode(double frequencyHz, double refClockHz)
  {
    double code = (frequencyHz / (refClockHz / 4.0)) * (1u << 27);
    if (code < 0) code = 0;
    if (code > 0xFFFFFF) code = 0xFFFFFF;
    return static_cast<uint32_t>(code);
  }

  /// Decode the two raw temperature bytes into degrees Celsius.
  /// 14-bit two's complement, 1/32 °C per LSB.
  inline double decodeTemperature(uint8_t msb, uint8_t lsb)
  {
    int32_t raw = ((static_cast<int32_t>(msb) & 0x3F) << 8) | lsb;
    if (msb & 0x20) // sign bit (bit 13)
    {
      raw -= 16384;
    }
    return raw / 32.0;
  }

  struct ImpedanceSample
  {
    double real;
    double imag;
    double magnitude;
    double phaseDeg;
  };

  /// Decode raw real/imag bytes (two's complement int16) and compute magnitude
  /// and phase in degrees in [-180, 180].
  inline ImpedanceSample decodeImpedance(uint8_t rMsb, uint8_t rLsb,
                                         uint8_t iMsb, uint8_t iLsb)
  {
    int16_t real = static_cast<int16_t>((rMsb << 8) | rLsb);
    int16_t imag = static_cast<int16_t>((iMsb << 8) | iLsb);
    double rd = real;
    double id = imag;
    ImpedanceSample s;
    s.real = rd;
    s.imag = id;
    s.magnitude = std::sqrt(rd * rd + id * id);
    s.phaseDeg = std::atan2(id, rd) * 180.0 / M_PI;
    return s;
  }

  /// Mid-point gain factor for a RES_ONLY calibration.
  inline double midpointGainFactor(double midMagnitude, double rCalOhm)
  {
    return 1.0 / (midMagnitude * rCalOhm);
  }

  /// Multi-point gain-factor slope in 1/(magnitude·Hz).
  inline double gainFactorSlope(double gfStart, double gfEnd,
                                double fStart, double fEnd)
  {
    return (gfEnd - gfStart) / (fEnd - fStart);
  }
}

#endif
