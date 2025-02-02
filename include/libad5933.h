#ifndef __LIB_AD5933_H
#define __LIB_AD5933_H

#include <libusb-1.0/libusb.h>
#include <vector>
#include <cassert>
#include <string>

#include "cyusb.h"

#include "ad5933types.h"

#define INIT_SETTERS_NUMBER  14

class Ad5933
{
private:
  libusb_device_handle* mpUsbHandle;
  UserParameters_st* mpUserParameters;

  bool mIsInit;
  bool mIsFirmwareDownloaded;
  bool mAreRegistersProgrammed;
  bool mIsGainFactorCalculated;

  double mGainFactor;
  double mDeltaGainFactorRate;

  Status_t mStatus;

  ImpedanceDataVector_t mImpedanceDataVector;

  inline int regWrite(uint8_t addr, uint8_t val);
  inline int regRead(uint8_t reg, uint8_t *val);
  
  void writeStartFrequency();
  void writeDeltaFrequency();
  void writeNumberOfIncrements();
  void writeNumSettlingTimeCycles();
  void writeSystemClock();
  void writeOutputExcitation();
  void writePgaControl();
  void writeFunction(Ad5933Function_t function);
  
  int readStatus();
  void pollStatus(unsigned int interval, unsigned int maxIter, uint8_t mask);
  void readImpedance();
  int reset();

public:
  Ad5933() :
    mpUsbHandle(nullptr), 
    mpUserParameters(nullptr),
    
    mIsInit(false),
    mIsFirmwareDownloaded(false),
    mAreRegistersProgrammed(false),
    mIsGainFactorCalculated(false),
    
    mGainFactor(0),
    mDeltaGainFactorRate(0),
    
    mStatus(0)
  {}

  ~Ad5933()
  {
    if(mpUsbHandle)
    {
      mpUsbHandle = nullptr; 
    }
    if(mpUserParameters)
    {
      mpUserParameters = nullptr;
    }
  }

  // Setter methods for SweepParameters
  void setRefClockFrequency(Frequency_t frequency) {mpUserParameters->mRefClockFrequency = frequency; }
  void setStartFrequency(Frequency_t frequency) { mpUserParameters->mStartFrequency = frequency; }
  void setDeltaFrequency(Frequency_t frequency) { mpUserParameters->mDeltaFrequency = frequency; }
  void setNumberOfIncrements(unsigned int increments) { mpUserParameters->mNumberOfIncrements = increments; }
  void setNumberSettlingTimeCycles(unsigned int cycles) { mpUserParameters->mNumberSettlingTimeCycles = cycles; }
  void setDdsSettlingTimeCycles(DdsSettlingTimeCycles_t cycles) { mpUserParameters->mDdsSettlingTimeCycles = cycles; }

  // Setter methods for SystemParameters
  void setClockConfiguration(ClockConfiguration_t config) { mpUserParameters->mClockConfiguration = config; }
  void setOutputExcitation(OutputExcitation_t excitation) { mpUserParameters->mOutputExcitation = excitation; }
  void setPgaControl(PgaControl_t control) { mpUserParameters->mPgaControl = control; }

  // Setter methods for CalibrationParameters
  void setCalibrationCircuitType(CalibrationCircuitType_t type) { mpUserParameters->mCalibrationCircuitType = type; }
  void setCalibrationMode(CalibrationMode_t mode) { mpUserParameters->mCalibrationMode = mode; }
  void setR1(ResistorValue_t r1) { mpUserParameters->mR1 = r1; }
  void setR2(ResistorValue_t r2) { mpUserParameters->mR2 = r2; }
  void setC1(CapacitorValue_t c1) { mpUserParameters->mC1 = c1; }

  // Getter methods for SweepParameters
  const Frequency_t& getRefClockFrequency() const { return mpUserParameters->mRefClockFrequency; }
  const Frequency_t& getStartFrequency() const { return mpUserParameters->mStartFrequency; }
  const Frequency_t& getDeltaFrequency() const { return mpUserParameters->DeltaFrequency; }
  const unsigned int& getNumberOfIncrements() const { return mpUserParameters->mNumberOfIncrements; }
  const unsigned int& getNumberSettlingTimeCycles() const { return mpUserParameters->mNumberSettlingTimeCycles; }
  const DdsSettlingTimeCycles_t& getDdsSettlingTimeCycles() const { return mpUserParameters->mDdsSettlingTimeCycles; }

  // Getter methods for SystemParameters
  const ClockConfiguration_t& getClockConfiguration() const { return mpUserParameters->mClockConfiguration; }
  const OutputExcitation_t& getOutputExcitation() const { return mpUserParameters->mOutputExcitation; }
  const PgaControl_t& getPgaControl() const { return mpUserParameters->mPgaControl; }

  // Getter methods for CalibrationParameters
  const CalibrationCircuitType_t& getCalibrationCircuitType() const { return mpUserParameters->mCalibrationCircuitType; }
  const CalibrationMode_t& getCalibrationMode() const { return mpUserParameters->mCalibrationMode; }
  const ResistorValue_t& getR1() const { return mpUserParameters->mR1; }
  const ResistorValue_t& getR2() const { return mpUserParameters->mR2; }
  const CapacitorValue_t& getC1() const { return mpUserParameters->mC1; }
  const GainFactor_t& getGainFactor() const { return mGainFactor; }
  const GainFactor_t& getDeltaGainFactor() const { return mDeltaGainFactorRate; }

  // Getter method for status
  const Status_t& getStatus() const { return mStatus; }
  
  // Getter method for impedance data vector
  const ImpedanceDataVector_t& getImpedanceDataVector() const { return  mImpedanceDataVector};

  // Device operations
  void connect(unsigned short vid, unsigned short pid); 
  void init(UserParameters_st& userParameters);
  void deinit();
  void readTemperature();
  void programDeviceRegisters();
  void startSweep();
  void doCalibration();
  void saveData(string filename);
};

#endif
