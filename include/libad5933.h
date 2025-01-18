#ifndef __LIB_AD5933_H
#define __LIB_AD5933_H

#include <libusb-1.0/libusb.h>

#include "cyusb.h"

#include "types.h"

class Ad5933
{
private:
  
  /**
   * \brief USB handle for the connected device.
   */
  libusb_device_handle* mpUsbHandle;

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

  SweepParameters_st msSweepParameters;

  // SYSTEM
  struct SystemParameters_st
  {
    ClockConfiguration_t mClockConfiguration;
    OutputExcitation_t mOutputExcitation;
    PgaControl_t mPgaControl;
    bool mAreRegistersProgrammed;
  };

  SystemParameters_st msSystemParameters;

  // CALIBRATION IMPEDANCE
  struct CalibrationParameters_st
  {
    CalibrationCircuitType_t mCalibrationCircuitType;
    CalibrationMode_t mCalibrationMode;
    ResistorValue_t mR1;
    ResistorValue_t mR2;
    CapacitorValue_t mC1;
    GainFactor_t mGainFactor;
    bool mIsGainFactorCalculated;
  };

  CalibrationParameters_st msCalibrationParameters;

  InternalTemperature_t mInternalTemperature;

public:
  Ad5933() : 
    mpUsbHandle(nullptr), 
    
    msSweepParameters
    { 
      .mRefClockFrequency = 0, 
      .mStartFrequency = 0, 
      .mDeltaFrequency = 0, 
      .mNumberOfIncrements = 0, 
      .mNumberSettlingTimeCycles = 0, 
      .mDdsSettlingTimeCycles = DdsSettlingTimeCycles_t::DEFAULT_X1
    },
      
    msSystemParameters
    { 
      .mClockConfiguration = ClockConfiguration_t::EXTERNAL_CLOCK, 
      .mOutputExcitation = OutputExcitation_t::RANGE_2VPP, 
      .mPgaControl = PgaControl_t::GAIN_X1, 
      .mAreRegistersProgrammed = false 
    },
      
    msCalibrationParameters
    { 
      .mCalibrationCircuitType = CalibrationCircuitType_t::RES_ONLY, 
      .mCalibrationMode = CalibrationMode_t::MID_POINT, 
      .mR1 = 0, 
      .mR2 = 0, 
      .mC1 = 0, 
      .mGainFactor = 0, 
      .mIsGainFactorCalculated = false 
    },

    mInternalTemperature(0)
  {}

  ~Ad5933()
  {
    if (mpUsbHandle)
    {
      mpUsbHandle = nullptr; 
    }
  }

  // Setter methods for SweepParameters
  void setRefClockFrequency(Frequency_t frequency) { msSweepParameters.mRefClockFrequency = frequency; }
  void setStartFrequency(Frequency_t frequency) { msSweepParameters.mStartFrequency = frequency; }
  void setDeltaFrequency(Frequency_t frequency) { msSweepParameters.mDeltaFrequency = frequency; }
  void setNumberOfIncrements(unsigned int increments) { msSweepParameters.mNumberOfIncrements = increments; }
  void setNumberSettlingTimeCycles(unsigned int cycles) { msSweepParameters.mNumberSettlingTimeCycles = cycles; }
  void setDdsSettlingTimeCycles(DdsSettlingTimeCycles_t cycles) { msSweepParameters.mDdsSettlingTimeCycles = cycles; }

  // Setter methods for SystemParameters
  void setClockConfiguration(ClockConfiguration_t config) { msSystemParameters.mClockConfiguration = config; }
  void setOutputExcitation(OutputExcitation_t excitation) { msSystemParameters.mOutputExcitation = excitation; }
  void setPgaControl(PgaControl_t control) { msSystemParameters.mPgaControl = control; }

  // Setter methods for CalibrationParameters
  void setCalibrationCircuitType(CalibrationCircuitType_t type) { msCalibrationParameters.mCalibrationCircuitType = type; }
  void setCalibrationMode(CalibrationMode_t mode) { msCalibrationParameters.mCalibrationMode = mode; }
  void setR1(ResistorValue_t r1) { msCalibrationParameters.mR1 = r1; }
  void setR2(ResistorValue_t r2) { msCalibrationParameters.mR2 = r2; }
  void setC1(CapacitorValue_t c1) { msCalibrationParameters.mC1 = c1; }
  void setGainFactor(GainFactor_t gain) { msCalibrationParameters.mGainFactor = gain; }
  void setGainFactorCalculated(bool calculated) { msCalibrationParameters.mIsGainFactorCalculated = calculated; }

  // Getter methods for SweepParameters
  const Frequency_t& getRefClockFrequency() const { return msSweepParameters.mRefClockFrequency; }
  const Frequency_t& getStartFrequency() const { return msSweepParameters.mStartFrequency; }
  const Frequency_t& getDeltaFrequency() const { return msSweepParameters.mDeltaFrequency; }
  const unsigned int& getNumberOfIncrements() const { return msSweepParameters.mNumberOfIncrements; }
  const unsigned int& getNumberSettlingTimeCycles() const { return msSweepParameters.mNumberSettlingTimeCycles; }
  const DdsSettlingTimeCycles_t& getDdsSettlingTimeCycles() const { return msSweepParameters.mDdsSettlingTimeCycles; }

  // Getter methods for SystemParameters
  const ClockConfiguration_t& getClockConfiguration() const { return msSystemParameters.mClockConfiguration; }
  const OutputExcitation_t& getOutputExcitation() const { return msSystemParameters.mOutputExcitation; }
  const PgaControl_t& getPgaControl() const { return msSystemParameters.mPgaControl; }
  const bool areRegistersProgrammed() const { return msSystemParameters.mAreRegistersProgrammed; }

  // Getter methods for CalibrationParameters
  const CalibrationCircuitType_t& getCalibrationCircuitType() const { return msCalibrationParameters.mCalibrationCircuitType; }
  const CalibrationMode_t& getCalibrationMode() const { return msCalibrationParameters.mCalibrationMode; }
  const ResistorValue_t& getR1() const { return msCalibrationParameters.mR1; }
  const ResistorValue_t& getR2() const { return msCalibrationParameters.mR2; }
  const CapacitorValue_t& getC1() const { return msCalibrationParameters.mC1; }
  const GainFactor_t& getGainFactor() const { return msCalibrationParameters.mGainFactor; }
  const bool isGainFactorCalculated() const { return msCalibrationParameters.mIsGainFactorCalculated; }

  // Getter method for InternalTemperature
  const InternalTemperature_t& getInternalTemperature() const { return mInternalTemperature; } 

};

#endif