#ifndef __TYPES_H
#define __TYPES_H

#include <stdint.h>

/**
 * \brief Structure to hold raw temperature data.
 */
typedef struct
{
  uint8_t temp_msb; /**< Most significant byte of the temperature data. */
  uint8_t temp_lsb; /**< Least significant byte of the temperature data. */
} stTemperatureRaw_t;

/**
 * \brief Type definition for processed temperature data.
 */
typedef double st_temperature;

/**
 * \brief Structure to hold raw impedance data.
 */
typedef struct
{
  uint8_t real_msb;
  uint8_t real_lsb;
  uint8_t imag_msb;
  uint8_t imag_lsb;
} stImpedDataRaw_t;

enum ClockConfiguration_t
{
  EXTERNAL_CLOCK,
  INTERNAL_CLOCK
};

enum OutputExcitation_t
{
  RANGE_2VPP,
  RANGE_1VPP,
  RANGE_04VPP,
  RANGE_02VPP
};

enum PgaControl_t
{
  GAIN_X1,
  GAIN_X5
};

enum CalibrationCircuitType_t
{
  RES_ONLY,
  CAP_ONLY,
  RES_CAP_SERIES,
  RES_CAP_PARALLEL,
  COMPLEX_CIRCUIT
};

enum DdsSettlingTimeCycles_t
{
  DEFAULT_X1,
  DOUBLE_X2,
  QUADRUPLE_X4
};

enum CalibrationMode_t
{
  MID_POINT,
  MULTI_POINT  
};

typedef double ResistorValue_t;
typedef double CapacitorValue_t;
typedef double Frequency_t;
typedef double GainFactor_t;
typedef double InternalTemperature_t;


#endif
