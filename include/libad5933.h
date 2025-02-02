#ifndef __LIB_AD5933_H
#define __LIB_AD5933_H

#include <libusb-1.0/libusb.h>
#include <vector>
#include <cassert>

#include "cyusb.h"

#include "types.h"

#define INIT_SETTERS_NUMBER  14

class Ad5933
{
private:
  std::vector<bool> settersCalled;
  libusb_device_handle* mpUsbHandle;
  bool mIsInit;
  bool mIsFirmwareDownloaded;
  Status_t mStatus;
  Ad5933Function_t mFunction;
  std::vector<ImpedData_ct> mImpedanceDataVector;

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
    double mDeltaGainFactorRate;
    bool mIsGainFactorCalculated;
  };

  CalibrationParameters_st msCalibrationParameters;

  TemperatureRaw_st msTemperatureRaw;
  Temperature_t     mTemperature;

  ImpedDataRaw_st msImpedDataRaw;
  ImpedData_ct mcImpedData;

  inline int regWrite(uint8_t addr, uint8_t val);
  inline int regRead(uint8_t reg, uint8_t *val);
  int readStatus();
  int reset();
  
  void writeStartFrequency();
  void writeDeltaFrequency();
  void writeNumberOfIncrements();
  void writeNumSettlingTimeCycles();

  void writeSystemClock();
  void writeOutputExcitation();
  void writePgaControl();

  void writeFunction(Ad5933Function_t function);
  void pollStatus(unsigned int interval, unsigned int maxIter, uint8_t mask);

  void readImpedance();

public:
  Ad5933() :
    settersCalled(INIT_SETTERS_NUMBER, false),
    mpUsbHandle(nullptr), 
    mIsInit(false),
    mFunction(Ad5933Function_t::NO_OPERATION),
    mImpedanceDataVector(),
    
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
      .mDeltaGainFactorRate = 0,
      .mIsGainFactorCalculated = false 
    },
    
    msTemperatureRaw
    {
      .temp_msb = 0,
      .temp_lsb = 0
    },
    
    mTemperature(0),

    msImpedDataRaw
    {
      .real_msb = 0,
      .real_lsb = 0,
      .imag_msb = 0,
      .imag_lsb = 0
    },

    mcImpedData()
  {}

  ~Ad5933()
  {
    if (mpUsbHandle)
    {
      mpUsbHandle = nullptr; 
    }
  }

  bool areAllSettersCalled() const
  {
    for (bool called : settersCalled)
    {
      if (!called) {
        return false; 
      }
    }
    return true;
  }

  void checkSetters() const
  {
    if (!areAllSettersCalled())
    {
      throw std::runtime_error("Not all required setters were called.");
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
  const GainFactor_t& getDeltaGainFactor() const { return msCalibrationParameters.mDeltaGainFactorRate; }
  const bool isGainFactorCalculated() const { return msCalibrationParameters.mIsGainFactorCalculated; }

  // Getter method for InternalTemperature
  const TemperatureRaw_st& getTemperatureRaw() const { return msTemperatureRaw; }
  const Temperature_t& getTemperature() const { return mTemperature; }

  // Getter for impedance calculation
  const ImpedDataRaw_st& getImpedRaw() const { return msImpedDataRaw; }
  const ImpedData_ct& getImped() const { return mcImpedData; }

  // Getter method for status
  const Status_t& getStatus() const { return mStatus; }
  // Getter method for function
  const Ad5933Function_t& getFunction() const {return mFunction; }

  // Device operations
  void init(unsigned short vid, unsigned short pid);
  void deinit();
  void readTemperature();
  void programDeviceRegisters();
  void startSweep();
  void doCalibration();
  void saveData();
};

#endif
