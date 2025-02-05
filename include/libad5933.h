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
  bool mAreDataCaptured;
  bool mIsGainFactorCalculated;

  double mGainFactor;
  double mDeltaGainFactorRate;

  Status_t mStatus;
  
  ImpedDataRaw_st msImpedDataRaw; 
  ImpedData_ct mcImpedData;
  ImpedanceDataVector_t mImpedanceDataVector;

  inline void regWrite(uint8_t addr, uint8_t val);
  inline void regRead(uint8_t reg, uint8_t *val);
  
  void writeStartFrequency();
  void writeDeltaFrequency();
  void writeNumberOfIncrements();
  void writeNumSettlingTimeCycles();
  void writeSystemClock();
  void writeOutputExcitation();
  void writePgaControl();
  void writeFunction(Ad5933Function_t function);
  
  void readStatus();
  void pollStatus(unsigned int interval, unsigned int maxIter, uint8_t mask);
  void readImpedance();
  void reset();

public:
  Ad5933() :
    mpUsbHandle(nullptr), 
    mpUserParameters(nullptr),
    
    mIsInit(false),
    mIsFirmwareDownloaded(false),
    mAreRegistersProgrammed(false),
    mAreDataCaptured(false),
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

  // Getter methods for SweepParameters
  const Frequency_t& getRefClockFrequency() const { return mpUserParameters->mRefClockFrequency; }
  const Frequency_t& getStartFrequency() const { return mpUserParameters->mStartFrequency; }
  const Frequency_t& getDeltaFrequency() const { return mpUserParameters->mDeltaFrequency; }
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
  const ImpedanceDataVector_t& getImpedanceDataVector() const { return  mImpedanceDataVector; }

  // Device operations
  void connect(unsigned short vid, unsigned short pid); 
  void init(UserParameters_st* userParameters);
  void deinit();
  Temperature_t readTemperature();
  void programDeviceRegisters();
  void startSweep();
  void doCalibration();
  void saveData(const char* filename);
};

#endif
