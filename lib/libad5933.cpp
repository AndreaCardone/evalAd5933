#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

#include "../include/libad5933.h"
#include "../include/ad5933math.h"

Ad5933::Ad5933()
  : mpUsbHandle(nullptr),
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

Ad5933::~Ad5933()
{
  mpUsbHandle = nullptr;
  mpUserParameters = nullptr;
}

// ----- Private register I/O -------------------------------------------------

void Ad5933::regWrite(uint8_t addr, uint8_t val)
{
  uint8_t dum = val;
  int res = libusb_control_transfer(mpUsbHandle, 0x40, 0xde, 0x0d,
                                    (val << 8) | addr, &dum, 1, 1000);
  if (res < 0)
  {
    throw std::runtime_error(libusb_strerror(res));
  }
}

void Ad5933::regRead(uint8_t reg, uint8_t* val)
{
  int res = libusb_control_transfer(mpUsbHandle, 0xc0, 0xde, 0x0d, reg, val, 1, 1000);
  if (res < 0)
  {
    throw std::runtime_error(libusb_strerror(res));
  }
}

void Ad5933::readStatus()
{
  regRead(AD5933_STATUS_REG, &mStatus);
}

void Ad5933::reset()
{
  uint8_t ctrl_lsb = 0;
  regRead(AD5933_CTRL_REG_LSB, &ctrl_lsb);
  ctrl_lsb |= MASK_CTRL_RESET;
  regWrite(AD5933_CTRL_REG_LSB, ctrl_lsb);
}

// ----- Register programming -------------------------------------------------

void Ad5933::writeStartFrequency()
{
  uint32_t code = ad5933::math::frequencyCode(
      mpUserParameters->mStartFrequency, mpUserParameters->mRefClockFrequency);
  regWrite(AD5933_START_FREQ1, (code >> 16) & 0xff);
  regWrite(AD5933_START_FREQ2, (code >>  8) & 0xff);
  regWrite(AD5933_START_FREQ3,  code        & 0xff);
}

void Ad5933::writeDeltaFrequency()
{
  uint32_t code = ad5933::math::frequencyCode(
      mpUserParameters->mDeltaFrequency, mpUserParameters->mRefClockFrequency);
  regWrite(AD5933_FREQ_INC1, (code >> 16) & 0xff);
  regWrite(AD5933_FREQ_INC2, (code >>  8) & 0xff);
  regWrite(AD5933_FREQ_INC3,  code        & 0xff);
}

void Ad5933::writeNumSettlingTimeCycles()
{
  uint8_t multiplier = 1;
  switch (mpUserParameters->mDdsSettlingTimeCycles)
  {
  case DEFAULT_X1:   multiplier = 1; break;
  case DOUBLE_X2:    multiplier = 2; break;
  case QUADRUPLE_X4: multiplier = 4; break;
  }

  uint8_t msb = (multiplier << 3) | ((mpUserParameters->mNumberSettlingTimeCycles >> 8) & 0x01);
  uint8_t lsb = mpUserParameters->mNumberSettlingTimeCycles & 0xff;

  regWrite(AD5933_NUMB_SETT_MSB, msb);
  regWrite(AD5933_NUMB_SETT_LSB, lsb);
}

void Ad5933::writeNumberOfIncrements()
{
  uint8_t msb = (getNumberOfIncrements() >> 8) & 0x01;
  uint8_t lsb = getNumberOfIncrements() & 0xff;
  regWrite(AD5933_NUMB_INCR_MSB, msb);
  regWrite(AD5933_NUMB_INCR_LSB, lsb);
}

void Ad5933::writeSystemClock()
{
  uint8_t r = 0;
  regRead(AD5933_CTRL_REG_LSB, &r);
  r &= ~MASK_CTRL_SYS_CLK_FIELD;
  r |= (mpUserParameters->mClockConfiguration == EXTERNAL_CLOCK)
           ? MASK_CTRL_EXT_SYS_CLK
           : MASK_CTRL_INT_SYS_CLK;
  regWrite(AD5933_CTRL_REG_LSB, r);
}

void Ad5933::writeOutputExcitation()
{
  uint8_t r = 0;
  regRead(AD5933_CTRL_REG_MSB, &r);
  r &= ~MASK_CTRL_OUT_V_RANGE;
  switch (mpUserParameters->mOutputExcitation)
  {
  case RANGE_2VPP:  r |= MASK_CTRL_2_V_PP;    break;
  case RANGE_1VPP:  r |= MASK_CTRL_1_V_PP;    break;
  case RANGE_04VPP: r |= MASK_CTRL_400_MV_PP; break;
  case RANGE_02VPP: r |= MASK_CTRL_200_MV_PP; break;
  }
  regWrite(AD5933_CTRL_REG_MSB, r);
}

void Ad5933::writePgaControl()
{
  uint8_t r = 0;
  regRead(AD5933_CTRL_REG_MSB, &r);
  r &= ~MASK_CTRL_PGA_FIELD;
  r |= (mpUserParameters->mPgaControl == GAIN_X1) ? MASK_CTRL_PGA_X1 : MASK_CTRL_PGA_X5;
  regWrite(AD5933_CTRL_REG_MSB, r);
}

void Ad5933::writeFunction(Ad5933Function_t function)
{
  static constexpr uint8_t map[] = {
      MASK_CTRL_NO_OPERATION,
      MASK_CTRL_INIT_START_FREQ,
      MASK_CTRL_START_FREQ_SWEEP,
      MASK_CTRL_INCR_FREQ,
      MASK_CTRL_REPEAT_FREQ,
      MASK_CTRL_NO_OPERATION_1,
      MASK_CTRL_MEASURE_TEMP,
      MASK_CTRL_PWR_DOWN_MODE,
      MASK_CTRL_STANDBY_MODE,
      MASK_CTRL_NO_OPERATION_2,
      MASK_CTRL_NO_OPERATION_3,
  };
  uint8_t r = 0;
  regRead(AD5933_CTRL_REG_MSB, &r);
  r &= ~MASK_CTRL_FUNCTION;
  r |= map[function];
  regWrite(AD5933_CTRL_REG_MSB, r);
}

void Ad5933::pollStatus(unsigned int intervalUs, unsigned int maxIter, uint8_t mask)
{
  for (unsigned int j = 0; j < maxIter; ++j)
  {
    readStatus();
    if ((mStatus & mask) == mask) return;
    usleep(intervalUs);
  }
  throw std::runtime_error("Polling failed: status mask never set.");
}

void Ad5933::readImpedance()
{
  regRead(AD5933_REAL_DATA_MSB, &msImpedDataRaw.real_msb);
  regRead(AD5933_REAL_DATA_LSB, &msImpedDataRaw.real_lsb);
  regRead(AD5933_IMAG_DATA_MSB, &msImpedDataRaw.imag_msb);
  regRead(AD5933_IMAG_DATA_LSB, &msImpedDataRaw.imag_lsb);

  auto s = ad5933::math::decodeImpedance(
      msImpedDataRaw.real_msb, msImpedDataRaw.real_lsb,
      msImpedDataRaw.imag_msb, msImpedDataRaw.imag_lsb);

  mcImpedData.m = s.magnitude;
  mcImpedData.phase = s.phaseDeg;

  if (mIsGainFactorCalculated)
  {
    double gf = mGainFactor;
    if (mpUserParameters->mCalibrationMode == MULTI_POINT)
    {
      gf = mGainFactor + mDeltaGainFactorRate *
                             (mcImpedData.frequency - mpUserParameters->mStartFrequency);
    }
    mcImpedData.magnitude = 1.0 / (s.magnitude * gf);
  }
  else
  {
    mcImpedData.magnitude = s.magnitude;
  }
}

// ----- Public API -----------------------------------------------------------

void Ad5933::connect(unsigned short vid, unsigned short pid, const std::string& firmwarePath)
{
  int res = cyusb_open(vid, pid);
  if (res != 1)
  {
    throw CyusbError("cyusb_open failed: " + std::to_string(res));
  }

  mpUsbHandle = cyusb_gethandle(0);

  res = cyusb_download_fx2(mpUsbHandle, const_cast<char*>(firmwarePath.c_str()), 0xA0);
  if (res != 0)
  {
    throw CyusbError("cyusb_download_fx2 failed: " + std::to_string(res));
  }
  mIsFirmwareDownloaded = true;
}

void Ad5933::setDeviceParameters(UserParameters_st* p)
{
  mIsInit = false;
  if (!p)
    throw std::invalid_argument("UserParameters pointer is null.");
  if (p->mRefClockFrequency <= 0)
    throw std::invalid_argument("Reference clock frequency must be positive.");
  if (p->mStartFrequency < 0 || p->mStartFrequency > 100000)
    throw std::invalid_argument("Start frequency must be in [0, 100000] Hz.");
  if (p->mDeltaFrequency < 0 || p->mDeltaFrequency > 100000)
    throw std::invalid_argument("Delta frequency must be in [0, 100000] Hz.");
  if (p->mNumberOfIncrements > MAX_INCREMENT_NUMBER)
    throw std::invalid_argument("Number of increments must be <= 511.");
  if (p->mNumberSettlingTimeCycles > MAX_SETTL_TIME_CYCLES)
    throw std::invalid_argument("Number of settling time cycles must be <= 511.");
  if (p->mCalibrationCircuitType != RES_ONLY)
    throw std::invalid_argument("Only RES_ONLY calibration is implemented.");
  if (p->mR1 < 0 || p->mR1 > 1000000)
    throw std::invalid_argument("R1 must be in [0, 1e6] Ohm.");

  mpUserParameters = p;
  mIsInit = true;
}

void Ad5933::deinit()
{
  cyusb_close();
  mpUsbHandle = nullptr;
  mIsInit = false;
}

Temperature_t Ad5933::readTemperature()
{
  TemperatureRaw_st raw;
  writeFunction(MEASURE_TEMPERATURE);
  pollStatus(100, 1000, MASK_STS_TEMP_VALID);
  regRead(AD5933_TEMP_DATA_MSB, &raw.temp_msb);
  regRead(AD5933_TEMP_DATA_LSB, &raw.temp_lsb);
  return ad5933::math::decodeTemperature(raw.temp_msb, raw.temp_lsb);
}

void Ad5933::programDeviceRegisters()
{
  mAreRegistersProgrammed = false;
  if (!mIsInit)
    throw std::logic_error("Cannot program registers before setDeviceParameters().");

  writeStartFrequency();
  writeDeltaFrequency();
  writeNumberOfIncrements();
  writeNumSettlingTimeCycles();
  writeSystemClock();
  writeOutputExcitation();
  writePgaControl();

  mAreRegistersProgrammed = true;
}

void Ad5933::startSweep()
{
  mAreDataCaptured = false;
  if (!mAreRegistersProgrammed)
    throw std::logic_error("Cannot start sweep before programDeviceRegisters().");

  mImpedanceDataVector.clear();
  mImpedanceDataVector.reserve(mpUserParameters->mNumberOfIncrements + 1);

  writeFunction(STANDBY_MODE);
  writeFunction(INIT_WITH_START_FREQ);
  writeFunction(START_FREQ_SWEEP);

  unsigned int i = 0;
  readStatus();
  while ((mStatus & MASK_STS_FRQ_SWP_DONE) != MASK_STS_FRQ_SWP_DONE)
  {
    pollStatus(100, 1000, MASK_STS_IMPED_VALID);
    mcImpedData.frequency =
        mpUserParameters->mStartFrequency + mpUserParameters->mDeltaFrequency * i;
    readImpedance();
    mImpedanceDataVector.push_back(mcImpedData);
    if (mProgressCb) mProgressCb(i, mcImpedData);
    writeFunction(INCREMENT_FREQUENCY);
    readStatus();
    ++i;
  }
  writeFunction(POWER_DOWN_MODE);
  mAreDataCaptured = true;
}

void Ad5933::doCalibration()
{
  mIsGainFactorCalculated = false;
  if (!mAreDataCaptured)
    throw std::logic_error("Cannot calibrate before a sweep has been captured.");
  if (mImpedanceDataVector.empty())
    throw std::logic_error("Empty impedance data vector.");

  if (mpUserParameters->mCalibrationCircuitType != RES_ONLY)
    throw std::logic_error("Only RES_ONLY calibration is implemented.");

  switch (mpUserParameters->mCalibrationMode)
  {
  case MID_POINT:
  {
    const auto& mid = mImpedanceDataVector[mImpedanceDataVector.size() / 2];
    mGainFactor = ad5933::math::midpointGainFactor(mid.m, mpUserParameters->mR1);
    break;
  }
  case MULTI_POINT:
  {
    if (mImpedanceDataVector.size() < 3)
      throw std::logic_error("Multi-point calibration needs >= 3 samples.");
    const auto& start = mImpedanceDataVector[1]; // skip first; sometimes unreliable
    const auto& end   = mImpedanceDataVector.back();
    double gfStart = ad5933::math::midpointGainFactor(start.m, mpUserParameters->mR1);
    double gfEnd   = ad5933::math::midpointGainFactor(end.m,   mpUserParameters->mR1);
    mGainFactor = gfStart;
    mDeltaGainFactorRate =
        ad5933::math::gainFactorSlope(gfStart, gfEnd, start.frequency, end.frequency);
    break;
  }
  }
  mIsGainFactorCalculated = true;
}

void Ad5933::saveData(const std::string& filename) const
{
  if (!mAreDataCaptured)
    throw std::logic_error("No data to save.");

  std::ofstream out(filename);
  if (!out)
    throw std::runtime_error("Cannot open file for writing: " + filename);

  out << "index,frequency_hz,magnitude,phase_deg\n";
  for (size_t i = 0; i < mImpedanceDataVector.size(); ++i)
  {
    const auto& s = mImpedanceDataVector[i];
    out << i << ',' << s.frequency << ',' << s.magnitude << ',' << s.phase << '\n';
  }
}
