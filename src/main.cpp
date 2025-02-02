#include <stdio.h>
#include <unistd.h>

#include "../include/libad5933.h"

#define VID 0x0456
#define PID 0xb203

int main()
{
  printf("\nProgram for EVAL-AD5933 by Andrea Cardone\n\n");

  Ad5933 ad5933;

  ad5933.connect(VID, PID);

  fprintf(stdout, "Initialization completed.\n\n");

  sleep(1);
  
  Temperature_t temp = 0;

  temp = ad5933.readTemperature();
  fprintf(stdout, "Read temperature: %f °C\n", temp);
  sleep(1);
 
  UserParameters_st userParameters
  {
    13000000,
    3500,     // start frequency
    50,       // delta frequency 
    100,      // number of increments
    50,       // number of settling time cycles
    DdsSettlingTimeCycles_t::DEFAULT_X1,
    ClockConfiguration_t::EXTERNAL_CLOCK,
    OutputExcitation_t::RANGE_1VPP,
    PgaControl_t::GAIN_X1,
    CalibrationCircuitType_t::RES_ONLY,
    CalibrationMode_t::MID_POINT,
    10000,
  };
  
  // initialize device parameters
  ad5933.init(&userParameters);

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
  ad5933.saveData("captured_data.csv");
  ad5933.deinit();
  return 0;
}
