#include <stdio.h>
#include <unistd.h>

#include "../include/libad5933.h"

#define VID 0x0456
#define PID 0xb203

int main()
{
  printf("\nProgram for EVAL-AD5933 by Andrea Cardone\n\n");

  Ad5933 ad5933;

  ad5933.init(VID, PID);

  fprintf(stdout, "Initialization completed.\n\n");

  sleep(1);
  
  Temperature_t temp = 0;

  ad5933.readTemperature();
  temp = ad5933.getTemperature();
  fprintf(stdout, "Read temperature: %f °C\n", temp);
  sleep(1);
  
  // settings
  ad5933.setStartFrequency(3500);
  ad5933.setDeltaFrequency(50);
  ad5933.setNumberOfIncrements(100);
  ad5933.setNumberSettlingTimeCycles(50);
  ad5933.setRefClockFrequency(16000000);
  ad5933.setClockConfiguration(ClockConfiguration_t::EXTERNAL_CLOCK);
  ad5933.setOutputExcitation(OutputExcitation_t::RANGE_1VPP);
  ad5933.setPgaControl(PgaControl_t::GAIN_X1);
  ad5933.setCalibrationCircuitType(CalibrationCircuitType_t::RES_ONLY);
  ad5933.setR1(10000);
  ad5933.setDdsSettlingTimeCycles(DdsSettlingTimeCycles_t::DEFAULT_X1);
  ad5933.setCalibrationMode(CalibrationMode_t::MID_POINT);

  // program device registers
  ad5933.programDeviceRegisters();
  
  fprintf(stdout, "Start sweep\n");
  ad5933.startSweep();
  //ad5933.saveData();
  fprintf(stdout, "Start calibration\n");
  ad5933.doCalibration();
  fprintf(stdout, "Start sweep\n");
  ad5933.programDeviceRegisters();
  ad5933.startSweep();
  ad5933.saveData();
  ad5933.deinit();
  return 0;
}