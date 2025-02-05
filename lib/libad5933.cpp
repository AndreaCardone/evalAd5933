#include <unistd.h>
#include <cmath>
#include "../include/libad5933.h"

// Private methods

inline void Ad5933::regWrite(uint8_t addr, uint8_t val)
{
  uint8_t dum = val & 0xff;
  int res = libusb_control_transfer(mpUsbHandle, 0x40, 0xde, 0x0d, val << 8 | addr, &dum, 1, 1000);
  if(res < 0)
  {
    throw std::runtime_error(libusb_strerror(res));
  }
}

inline void Ad5933::regRead(uint8_t reg, uint8_t* val)
{
  int res = libusb_control_transfer(mpUsbHandle, 0xc0, 0xde, 0x0d, reg, val, 1, 1000);
  if(res < 0)
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

void Ad5933::writeStartFrequency()
{
  unsigned int freq_code = 0;
  uint8_t f1 = 0;
  uint8_t f2 = 0;
  uint8_t f3 = 0;

  freq_code = static_cast<unsigned int>((mpUserParameters->mStartFrequency/(mpUserParameters->mRefClockFrequency/4))*pow(2,27));

  f1 = (freq_code >> 16) & 0xff;
  f2 = (freq_code >> 8) & 0xff;
  f3 = (freq_code) & 0xff;

  regWrite(AD5933_START_FREQ1, f1);
  regWrite(AD5933_START_FREQ2, f2);
  regWrite(AD5933_START_FREQ3, f3);
}

void Ad5933::writeDeltaFrequency()
{
  unsigned int step_code = 0;
  uint8_t f1 = 0;
  uint8_t f2 = 0;
  uint8_t f3 = 0;
  int ret = 0;

  step_code = static_cast<unsigned int>(mpUserParameters->mDeltaFrequency/(mpUserParameters->mRefClockFrequency/4)*pow(2,27));

  f1 = (step_code >> 16) & 0xff;
  f2 = (step_code >> 8) & 0xff;
  f3 = (step_code) & 0xff;

  regWrite(AD5933_FREQ_INC1, f1);
  regWrite(AD5933_FREQ_INC2, f2);
  regWrite(AD5933_FREQ_INC3, f3);
}

void Ad5933::writeNumSettlingTimeCycles()
{
  uint8_t multiplier;
  switch (mpUserParameters->mDdsSettlingTimeCycles)
  {
  case DEFAULT_X1:
    multiplier = 1;
    break;
  case DOUBLE_X2:
    multiplier = 2;
    break;
  case QUADRUPLE_X4:
    multiplier = 4;
  default:
    multiplier = 1;
    break;
  }

  uint8_t msb = (multiplier << 3) | ( (mpUserParameters->mNumberSettlingTimeCycles >> 8) & 0xff );
  uint8_t lsb = mpUserParameters->mNumberSettlingTimeCycles & 0xff;

  regWrite(AD5933_NUMB_SETT_MSB, msb);
  regWrite(AD5933_NUMB_SETT_LSB, lsb);
}

void Ad5933::writeNumberOfIncrements()
{
  uint8_t msb = (getNumberOfIncrements() >> 8) & 0xff;
  uint8_t lsb = getNumberOfIncrements() & 0xff;
  regWrite(AD5933_NUMB_INCR_MSB, msb);
  regWrite(AD5933_NUMB_INCR_LSB, lsb);
}

void Ad5933::writeSystemClock()
{
  uint8_t control_reg_lsb = 0;
  regRead(AD5933_CTRL_REG_LSB, &control_reg_lsb);
  
  switch (mpUserParameters->mClockConfiguration)
  {
  case EXTERNAL_CLOCK:
    (control_reg_lsb &= ~MASK_CTRL_SYS_CLK_FIELD) |= MASK_CTRL_EXT_SYS_CLK;
    break;
  case INTERNAL_CLOCK:
    (control_reg_lsb &= ~MASK_CTRL_SYS_CLK_FIELD) |= MASK_CTRL_INT_SYS_CLK;
    break;
  default:
    (control_reg_lsb &= ~MASK_CTRL_SYS_CLK_FIELD) |= MASK_CTRL_EXT_SYS_CLK;
    break;
  }

  regWrite(AD5933_CTRL_REG_LSB, control_reg_lsb);
}

void Ad5933::writeOutputExcitation()
{
  uint8_t control_reg_msb = 0;
  regRead(AD5933_CTRL_REG_MSB, &control_reg_msb);
  switch (mpUserParameters->mOutputExcitation)
  {
  case RANGE_2VPP:
    (control_reg_msb &= ~MASK_CTRL_OUT_V_RANGE) |= MASK_CTRL_2_V_PP;
    break;
  case RANGE_1VPP:
    (control_reg_msb &= ~MASK_CTRL_OUT_V_RANGE) |= MASK_CTRL_1_V_PP;
    break;
  case RANGE_04VPP:
    (control_reg_msb &= ~MASK_CTRL_OUT_V_RANGE) |= MASK_CTRL_400_MV_PP;
    break;
  case RANGE_02VPP:
    (control_reg_msb &= ~MASK_CTRL_OUT_V_RANGE) |= MASK_CTRL_200_MV_PP;
    break;
  default:
    (control_reg_msb &= ~MASK_CTRL_OUT_V_RANGE) |= MASK_CTRL_2_V_PP;
    break;
  }
  regWrite(AD5933_CTRL_REG_MSB, control_reg_msb);
}

void Ad5933::writePgaControl()
{
  uint8_t control_reg_msb = 0;
  regRead(AD5933_CTRL_REG_MSB, &control_reg_msb);
  switch (mpUserParameters->mPgaControl)
  {
  case GAIN_X1:
    (control_reg_msb &= ~MASK_CTRL_PGA_FIELD) |= MASK_CTRL_PGA_X1;
    break;
  case GAIN_X5:
    (control_reg_msb &= ~MASK_CTRL_PGA_FIELD) |= MASK_CTRL_PGA_X5;
    break;
  default:
    (control_reg_msb &= ~MASK_CTRL_PGA_FIELD) |= MASK_CTRL_PGA_X1;
    break;
  }
  regWrite(AD5933_CTRL_REG_MSB, control_reg_msb);
}

void Ad5933::writeFunction(Ad5933Function_t function)
{
  uint8_t control_reg_msb = 0;
  regRead(AD5933_CTRL_REG_MSB, &control_reg_msb);
  switch (function)
  {
  case NO_OPERATION:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_NO_OPERATION;
    break;
  case INIT_WITH_START_FREQ:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_INIT_START_FREQ;
    break;
  case START_FREQ_SWEEP:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_START_FREQ_SWEEP;
    break;
  case INCREMENT_FREQUENCY:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_INCR_FREQ;
    break;
  case REPEAT_FREQUENCY:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_REPEAT_FREQ;
    break;
  case NO_OPERATION_1:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_NO_OPERATION_1;
    break;
  case MEASURE_TEMPERATURE:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_MEASURE_TEMP;
    break;
  case POWER_DOWN_MODE:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_PWR_DOWN_MODE;
    break;
  case STANDBY_MODE:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_STANDBY_MODE;
    break;
  case NO_OPERATION_2:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_NO_OPERATION_2;
    break;
  case NO_OPERATION_3:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_NO_OPERATION_3;
    break;
  default:
    (control_reg_msb &= ~MASK_CTRL_FUNCTION) |= MASK_CTRL_NO_OPERATION;
    break;
  }
  regWrite(AD5933_CTRL_REG_MSB, control_reg_msb);
}

void Ad5933::pollStatus(unsigned int interval, unsigned int maxIter, uint8_t mask)
{
  unsigned int j = 0;  
  while ((getStatus() & mask) != mask)
	{
    usleep(interval);
    readStatus();
    if (j == maxIter)
		{
		  throw std::runtime_error("Polling failed.");
		}
    j++;
  }
}

void Ad5933::readImpedance()
{
  regRead(AD5933_REAL_DATA_MSB, &(msImpedDataRaw.real_msb));
  regRead(AD5933_REAL_DATA_LSB, &(msImpedDataRaw.real_lsb));
  regRead(AD5933_IMAG_DATA_MSB, &(msImpedDataRaw.imag_msb));
  regRead(AD5933_IMAG_DATA_LSB, &(msImpedDataRaw.imag_lsb));

  double real = (msImpedDataRaw.real_msb << 8) | (msImpedDataRaw.real_lsb);
  double imag = (msImpedDataRaw.imag_msb << 8) | (msImpedDataRaw.imag_lsb);
  double module = sqrt(pow(real,2) + pow(imag,2));
  double phase = 0;

  if ((real > 0) && (imag > 0)) 
  {          
    phase = atan(imag/real)*180/M_PI; // first quadrant
  }
  else if ((real < 0) && (imag > 0))
  {    
    phase = atan(imag/real)*180/M_PI + 180; // second quadrant
  }
  else if ((real < 0) && (imag < 0)) 
  {     
    phase = atan(imag/real)*180/M_PI - 180; // third quadrant
  }
  else                             
  {       
    phase = atan(imag/real)*180/M_PI + 360; // fourth quadrant
  }

  mcImpedData.m = module;
  mcImpedData.phase = phase;
  if (mIsGainFactorCalculated)
  {
    switch (mpUserParameters->mCalibrationMode)
    {
    case CalibrationMode_t::MID_POINT:
    {
      mcImpedData.magnitude = 1/(module*mGainFactor);
      break;
    }
    case CalibrationMode_t::MULTI_POINT:
    {
      double pointGainFactor = mGainFactor + mDeltaGainFactorRate*(mcImpedData.frequency - mpUserParameters->mStartFrequency);
      mcImpedData.magnitude = 1/(module*mGainFactor);
      break;
    }
    default:
      break;
    }
  }
  else
  {
    mcImpedData.magnitude = module;
  }
}
// Public methods

void Ad5933::connect(unsigned short vid, unsigned short pid)
{
  int res;
  res = cyusb_open(vid, pid);
  if(res != 1)
  {
    throw CyusbError("Error in opening device, cyusb_open: " + std::to_string(res) + "\n");
  }

  mpUsbHandle = cyusb_gethandle(0);

  res = cyusb_download_fx2(mpUsbHandle, (char*)AD5933_FW_PATH, 0xA0);

  if(res == 0)
  {
    mIsFirmwareDownloaded = true;
  }
  else
  {
    throw CyusbError("Unable to download firmware, cyusb_download_fx2: " + std::to_string(res) + "\n");
  }
}

void Ad5933::init(UserParameters_st* pUserParameters)
{
  mIsInit = false;
  if(pUserParameters == nullptr)
  {
    std::cerr << "Error: Pointer to struct is null!" << std::endl;
    return;
  }
 
  if(pUserParameters->mRefClockFrequency > 0)
  {
    std::cerr << "Error: Reference clock frequency cannot be nagative!" << std::endl;
    return;
  }

  if(pUserParameters->mStartFrequency < 0 || pUserParameters->mStartFrequency > 100000)
  {
    std::cerr << "Error valid start frequency must be between 0 and 100 kHz!" << std::endl;
    return;
  }

  if(pUserParameters->mDeltaFrequency < 0 || pUserParameters->mDeltaFrequency > 100000)
  {
    std::cerr << "Error: Delta frequency must be between 0 and 100 kHz!" << std::endl;
    return;
  }

  if(pUserParameters->mNumberOfIncrements < 0 || pUserParameters->mNumberOfIncrements > 511)
  {
    std::cerr << "Error: Number of increments must be between 0 and 511 (9 bits)!" << std::endl;
    return;
  }

  if(pUserParameters->mNumberSettlingTimeCycles < 0 || pUserParameters->mNumberSettlingTimeCycles > 511)
  {
    std::cerr << "Error: Number of settling time cycles must be between 0 and 511 (9 bits)!" << std::endl;
    return;
  }
  
  if(pUserParameters->mCalibrationCircuitType != CalibrationCircuitType_t::RES_ONLY)
  {
    std::cerr << "Error: Calibration circuit type not supported!" << std::endl;
    return;
  }

  if(pUserParameters->mR1 < 0 || pUserParameters->mR1 > 1000000)
  {
    std::cerr << "Error: R1 must be between 0 and 1 MOhm!" << std::endl;
  }

  mpUserParameters = pUserParameters;
  mIsInit = true;
}

void Ad5933::deinit()
{
  cyusb_close();
  if(!mpUsbHandle)
  {
    mpUsbHandle = nullptr;
  }
  if(mIsInit)
  {
    mIsInit = false;
  }
}

Temperature_t Ad5933::readTemperature()
{
  
  TemperatureRaw_st temperatureRaw;
  Temperature_t temperature;
  
  uint16_t temp = 0x0;
  uint8_t t0 = 0;
  uint8_t t1 = 0;
  uint8_t control_reg_msb = 0x0;

  writeFunction(Ad5933Function_t::MEASURE_TEMPERATURE);

  int i = 0;
  readStatus();
  pollStatus(100, 1000, MASK_STS_TEMP_VALID);
  regRead(AD5933_TEMP_DATA_MSB, &temperatureRaw.temp_msb);
  regRead(AD5933_TEMP_DATA_LSB, &temperatureRaw.temp_lsb);

  if (temperatureRaw.temp_msb >> 5)
    temperature = ((((temperatureRaw.temp_msb & 0xff) << 8) | (temperatureRaw.temp_lsb & 0xff)) - 16384) >> 5;
  else
    temperature = (((temperatureRaw.temp_msb & 0xff) << 8) | (temperatureRaw.temp_lsb & 0xff)) >> 5;
  
  return temperature;
}

void Ad5933::programDeviceRegisters()
{
  mAreRegistersProgrammed = false;
  
  if(!mIsInit)
  {
    std::cerr << "Error: Cannot program device registers if register values are not set properly!" << std::endl;
    return;
  }

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
  if(!mAreRegistersProgrammed)
  {
    std::cerr << "Error: Cannot start sweep if device registers are not programmed correctly" << std::endl;
    return;
  }

  mImpedanceDataVector.clear();
  writeFunction(Ad5933Function_t::STANDBY_MODE);
  writeFunction(Ad5933Function_t::INIT_WITH_START_FREQ);
  writeFunction(Ad5933Function_t::START_FREQ_SWEEP);

  unsigned int i = 0;
  readStatus();
  while ((getStatus() & MASK_STS_FRQ_SWP_DONE) != MASK_STS_FRQ_SWP_DONE)
  {
    pollStatus(100, 1000, MASK_STS_IMPED_VALID);
    mcImpedData.frequency = mpUserParameters->mStartFrequency + mpUserParameters->mDeltaFrequency*i;
    readImpedance();
    printf("%d\n", i);
    mImpedanceDataVector.push_back(mcImpedData);
    mcImpedData.display();
    printf("\n");
    writeFunction(Ad5933Function_t::INCREMENT_FREQUENCY);
    readStatus();
    i++;
  }
  writeFunction(Ad5933Function_t::POWER_DOWN_MODE);
  mAreDataCaptured = true;
}

void Ad5933::doCalibration()
{
  assert(!mImpedanceDataVector.empty());

  ImpedData_ct midpointImpedance;
  ImpedData_ct startpointImpedance;
  ImpedData_ct endpointImpedance;
  switch (mpUserParameters->mCalibrationCircuitType)
  case CalibrationCircuitType_t::RES_ONLY:
  {
    switch (mpUserParameters->mCalibrationMode)
    {
    case CalibrationMode_t::MID_POINT :
    {
      midpointImpedance = mImpedanceDataVector[static_cast<int>(mImpedanceDataVector.size()/2)];
      mGainFactor = 1/(midpointImpedance.m * mpUserParameters->mR1);
      break;
    }
    case CalibrationMode_t::MULTI_POINT:
    {
      startpointImpedance = mImpedanceDataVector[1]; // sometimes first sample may not be reliable
      endpointImpedance = mImpedanceDataVector[static_cast<int>(mImpedanceDataVector.size()-1)];
      //endpointImpedance = mImpedanceDataVector[2];
      double startpointGf = 1/(startpointImpedance.m * mpUserParameters->mR1);
      double endpointGf = 1/(endpointImpedance.m * mpUserParameters->mR1);
      mGainFactor = startpointGf;
      mDeltaGainFactorRate = (endpointGf - startpointGf)/(endpointImpedance.frequency - startpointImpedance.frequency);
      break;
    }
    default:
      throw std::invalid_argument("Unknown calibration mode.");
    }
    break;
  case CalibrationCircuitType_t::CAP_ONLY:
  case CalibrationCircuitType_t::RES_CAP_SERIES:
  case CalibrationCircuitType_t::RES_CAP_PARALLEL:
  case CalibrationCircuitType_t::COMPLEX_CIRCUIT:
    throw std::invalid_argument("Calibration circuit type not yet supported.");
    break;
  default:
    throw std::invalid_argument("Unknown calibration circuit type.");
    break;
  }
  mIsGainFactorCalculated = true;
}

void Ad5933::saveData(const char* filename)
{
  FILE *fd;

	if((fd = fopen(filename, "a+")) == NULL)
	{
		throw std::runtime_error("Error opening file for writing!\n");
	}

  int t = 0;
  ImpedData_ct imped;
  for(unsigned int i = 0; i < mImpedanceDataVector.size(); i++)
  {
    imped = mImpedanceDataVector[i];
    fprintf(fd, "%d,%f,%f,%f\n", t, imped.frequency, imped.magnitude, imped.phase);
    t++;
  }

	fclose(fd);
}
