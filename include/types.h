#ifndef __TYPES_H
#define __TYPES_H

#include <stdint.h>
#include <iostream>
#include <stdexcept>

// Registers
#define AD5933_CTRL_REG_MSB        0x80
#define AD5933_CTRL_REG_LSB        0x81
#define AD5933_START_FREQ1         0x82
#define AD5933_START_FREQ2         0x83
#define AD5933_START_FREQ3         0x84
#define AD5933_FREQ_INC1           0x85
#define AD5933_FREQ_INC2           0x86
#define AD5933_FREQ_INC3           0x87
#define AD5933_NUMB_INCR_MSB       0x88
#define AD5933_NUMB_INCR_LSB       0x89
#define AD5933_NUMB_SETT_MSB       0x8A
#define AD5933_NUMB_SETT_LSB       0x8B
#define AD5933_STATUS_REG          0x8f
#define AD5933_TEMP_DATA_MSB       0x92
#define AD5933_TEMP_DATA_LSB       0x93
#define AD5933_REAL_DATA_MSB       0x94
#define AD5933_REAL_DATA_LSB       0x95
#define AD5933_IMAG_DATA_MSB       0x96
#define AD5933_IMAG_DATA_LSB       0x97

// Control register 0x80
#define MASK_CTRL_FUNCTION         0xF0
#define MASK_CTRL_NO_OPERATION     0x00
#define MASK_CTRL_INIT_START_FREQ  0x10
#define MASK_CTRL_START_FREQ_SWEEP 0x20
#define MASK_CTRL_INCR_FREQ        0x30
#define MASK_CTRL_REPEAT_FREQ      0x40
#define MASK_CTRL_NO_OPERATION_1   0x80
#define MASK_CTRL_MEASURE_TEMP     0x90
#define MASK_CTRL_PWR_DOWN_MODE    0xA0
#define MASK_CTRL_STANDBY_MODE     0xB0
#define MASK_CTRL_NO_OPERATION_2   0xC0
#define MASK_CTRL_NO_OPERATION_3   0xD0

#define MASK_CTRL_OUT_V_RANGE      0x0C
#define MASK_CTRL_2_V_PP           0x00
#define MASK_CTRL_200_MV_PP        0x02
#define MASK_CTRL_400_MV_PP        0x04
#define MASK_CTRL_1_V_PP           0x06

#define MASK_CTRL_PGA_FIELD        0x01
#define MASK_CTRL_PGA_X5           0x00
#define MASK_CTRL_PGA_X1           0x01

// Control register 0x81
#define MASK_CTRL_RESET            0x10
#define MASK_CTRL_SYS_CLK_FIELD    0x08
#define MASK_CTRL_EXT_SYS_CLK      0x08
#define MASK_CTRL_INT_SYS_CLK      0x00

// Status register 0x8F
#define MASK_STS_TEMP_VALID        0x01
#define MASK_STS_IMPED_VALID       0x02
#define MASK_STS_FRQ_SWP_DONE      0x04

#define MAX_INCREMENT_NUMBER  511
#define MAX_SETTL_TIME_CYCLES 511

char const* AD5933_FW_PATH = "../cypressfw/AD5933_34FW.hex";

/**
 * \brief Structure to hold raw temperature data.
 */
typedef struct
{
  uint8_t temp_msb; /**< Most significant byte of the temperature data. */
  uint8_t temp_lsb; /**< Least significant byte of the temperature data. */
} TemperatureRaw_st;

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
} ImpedDataRaw_st;

class ImpedData_ct {
public:
    double magnitude; /**< Magnitude of the impedance. */
    double phase; /**< Phase of the impedance. */
    double m; /**< Additional parameter for impedance. */
    double frequency;

    ImpedData_ct() : magnitude(0), phase(0), m(0), frequency(0) {}
    ImpedData_ct(double mag, double ph, double mt, double freq)
        : magnitude(mag), phase(ph), m(mt), frequency(freq) {}
    ImpedData_ct(const ImpedData_ct &other)
        : magnitude(other.magnitude), phase(other.phase), m(other.m), frequency(other.frequency){}

    ImpedData_ct& operator=(const ImpedData_ct &other) {
      if (this == &other) return *this; // Handle self-assignment
      magnitude = other.magnitude;
      phase = other.phase;
      m = other.m;
      frequency = other.frequency;
      return *this;
    }

    ~ImpedData_ct() {}

    void display() const {
        std::cout << "Magnitude: " << magnitude << "\n"
                  << "Phase: " << phase << "\n"
                  << "M: " << m << "\n"
                  << "Frequency: " << frequency << std::endl;
    }
};

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

enum Ad5933Function_t
{
  NO_OPERATION,
  INIT_WITH_START_FREQ,
  START_FREQ_SWEEP,
  INCREMENT_FREQUENCY,
  REPEAT_FREQUENCY,
  NO_OPERATION_1,
  MEASURE_TEMPERATURE,
  POWER_DOWN_MODE,
  STANDBY_MODE,
  NO_OPERATION_2,
  NO_OPERATION_3
};

typedef double ResistorValue_t;
typedef double CapacitorValue_t;
typedef double Frequency_t;
typedef double GainFactor_t;
typedef double Temperature_t;
typedef uint8_t Status_t;

// SWEEP PARAMETERS
struct SweepParameters_st
{
  Frequency_t mRefClockFrequency; // Hz
  Frequency_t mStartFrequency; // Hz
  Frequency_t mDeltaFrequency; // Hz
  unsigned int mNumberOfIncrements; // 9 bits
  unsigned int mNumberSettlingTimeCycles;
  DdsSettlingTimeCycles_t mDdsSettlingTimeCycles;
};


// SYSTEM
struct SystemParameters_st
{
  ClockConfiguration_t mClockConfiguration;
  OutputExcitation_t mOutputExcitation;
  PgaControl_t mPgaControl;
  bool mAreRegistersProgrammed;
};


// CALIBRATION IMPEDANCE
struct CalibrationParameters_st
{
  CalibrationCircuitType_t mCalibrationCircuitType;
  CalibrationMode_t mCalibrationMode;
  ResistorValue_t mR1;
  ResistorValue_t mR2;
  CapacitorValue_t mC1;
  GainFactor_t mGainFactor;
  double mDeltaGainFactorRate;
  bool mIsGainFactorCalculated;
};


class CyusbError : public std::runtime_error {
public:
  CyusbError(const std::string& message) : std::runtime_error(message) {}
};

#endif
