#ifndef __LIB_AD5933_H
#define __LIB_AD5933_H

#include <libusb-1.0/libusb.h>
#include <vector>
#include <string>
#include <functional>

#include "cyusb.h"

#include "ad5933types.h"

#ifndef AD5933_DEFAULT_FW_PATH
#define AD5933_DEFAULT_FW_PATH "../cypressfw/AD5933_34FW.hex"
#endif

class Ad5933
{
public:
  using ProgressCallback = std::function<void(unsigned int index, const ImpedData_ct& sample)>;

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

  ProgressCallback mProgressCb;

  void regWrite(uint8_t addr, uint8_t val);
  void regRead(uint8_t reg, uint8_t* val);

  void writeStartFrequency();
  void writeDeltaFrequency();
  void writeNumberOfIncrements();
  void writeNumSettlingTimeCycles();
  void writeSystemClock();
  void writeOutputExcitation();
  void writePgaControl();
  void writeFunction(Ad5933Function_t function);

  void readStatus();
  void pollStatus(unsigned int intervalUs, unsigned int maxIter, uint8_t mask);
  void readImpedance();
  void reset();

public:
  Ad5933();
  ~Ad5933();

  Frequency_t getRefClockFrequency() const { return mpUserParameters->mRefClockFrequency; }
  Frequency_t getStartFrequency() const { return mpUserParameters->mStartFrequency; }
  Frequency_t getDeltaFrequency() const { return mpUserParameters->mDeltaFrequency; }
  unsigned int getNumberOfIncrements() const { return mpUserParameters->mNumberOfIncrements; }
  unsigned int getNumberSettlingTimeCycles() const { return mpUserParameters->mNumberSettlingTimeCycles; }
  DdsSettlingTimeCycles_t getDdsSettlingTimeCycles() const { return mpUserParameters->mDdsSettlingTimeCycles; }

  ClockConfiguration_t getClockConfiguration() const { return mpUserParameters->mClockConfiguration; }
  OutputExcitation_t getOutputExcitation() const { return mpUserParameters->mOutputExcitation; }
  PgaControl_t getPgaControl() const { return mpUserParameters->mPgaControl; }

  CalibrationCircuitType_t getCalibrationCircuitType() const { return mpUserParameters->mCalibrationCircuitType; }
  CalibrationMode_t getCalibrationMode() const { return mpUserParameters->mCalibrationMode; }
  ResistorValue_t getR1() const { return mpUserParameters->mR1; }
  ResistorValue_t getR2() const { return mpUserParameters->mR2; }
  CapacitorValue_t getC1() const { return mpUserParameters->mC1; }
  GainFactor_t getGainFactor() const { return mGainFactor; }
  GainFactor_t getDeltaGainFactor() const { return mDeltaGainFactorRate; }

  Status_t getStatus() const { return mStatus; }
  const ImpedanceDataVector_t& getImpedanceDataVector() const { return mImpedanceDataVector; }

  void setProgressCallback(ProgressCallback cb) { mProgressCb = std::move(cb); }

  // Device operations. Throw on error.
  void connect(unsigned short vid, unsigned short pid,
               const std::string& firmwarePath = AD5933_DEFAULT_FW_PATH);
  void setDeviceParameters(UserParameters_st* userParameters);
  void deinit();
  Temperature_t readTemperature();
  void programDeviceRegisters();
  void startSweep();
  void doCalibration();
  void saveData(const std::string& filename) const;
};

#endif
