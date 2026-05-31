#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>

#include "../include/libad5933.h"

#define VID 0x0456
#define PID 0xb203

namespace
{

struct CliOptions
{
  double refClock      = 13000000.0;
  double startFreq     = 3500.0;
  double deltaFreq     = 50.0;
  unsigned int steps   = 100;
  unsigned int settle  = 50;
  double r1            = 10000.0;
  std::string output   = "captured_data.csv";
  std::string fwPath   = AD5933_DEFAULT_FW_PATH;
  OutputExcitation_t vrange = RANGE_1VPP;
  PgaControl_t pga          = GAIN_X1;
  ClockConfiguration_t clk  = EXTERNAL_CLOCK;
  bool quiet = false;
};

void usage(const char* argv0)
{
  std::cout
    << "Usage: " << argv0 << " [options]\n"
    << "  --start <Hz>        start frequency (default 3500)\n"
    << "  --delta <Hz>        frequency increment (default 50)\n"
    << "  --steps <n>         number of increments, <=511 (default 100)\n"
    << "  --settle <n>        number of settling cycles, <=511 (default 50)\n"
    << "  --refclk <Hz>       reference clock (default 13000000)\n"
    << "  --r1 <Ohm>          calibration resistor (default 10000)\n"
    << "  --vrange <2|1|0.4|0.2>  output excitation Vpp (default 1)\n"
    << "  --pga <1|5>         PGA gain (default 1)\n"
    << "  --clk <int|ext>     clock source (default ext)\n"
    << "  --fw <path>         firmware hex path\n"
    << "  --output <file>     CSV output path (default captured_data.csv)\n"
    << "  --quiet             suppress per-sample progress\n"
    << "  -h, --help          this message\n";
}

bool parse(int argc, char** argv, CliOptions& o)
{
  auto need = [&](int& i) {
    if (i + 1 >= argc) { std::cerr << "Missing value for " << argv[i] << "\n"; return false; }
    ++i; return true;
  };
  for (int i = 1; i < argc; ++i)
  {
    std::string a = argv[i];
    if      (a == "-h" || a == "--help") { usage(argv[0]); std::exit(0); }
    else if (a == "--start")   { if(!need(i)) return false; o.startFreq = std::atof(argv[i]); }
    else if (a == "--delta")   { if(!need(i)) return false; o.deltaFreq = std::atof(argv[i]); }
    else if (a == "--steps")   { if(!need(i)) return false; o.steps     = std::atoi(argv[i]); }
    else if (a == "--settle")  { if(!need(i)) return false; o.settle    = std::atoi(argv[i]); }
    else if (a == "--refclk")  { if(!need(i)) return false; o.refClock  = std::atof(argv[i]); }
    else if (a == "--r1")      { if(!need(i)) return false; o.r1        = std::atof(argv[i]); }
    else if (a == "--output")  { if(!need(i)) return false; o.output    = argv[i]; }
    else if (a == "--fw")      { if(!need(i)) return false; o.fwPath    = argv[i]; }
    else if (a == "--quiet")   { o.quiet = true; }
    else if (a == "--vrange")
    {
      if (!need(i)) return false;
      std::string v = argv[i];
      if      (v == "2")   o.vrange = RANGE_2VPP;
      else if (v == "1")   o.vrange = RANGE_1VPP;
      else if (v == "0.4") o.vrange = RANGE_04VPP;
      else if (v == "0.2") o.vrange = RANGE_02VPP;
      else { std::cerr << "Bad --vrange: " << v << "\n"; return false; }
    }
    else if (a == "--pga")
    {
      if (!need(i)) return false;
      std::string v = argv[i];
      if      (v == "1") o.pga = GAIN_X1;
      else if (v == "5") o.pga = GAIN_X5;
      else { std::cerr << "Bad --pga: " << v << "\n"; return false; }
    }
    else if (a == "--clk")
    {
      if (!need(i)) return false;
      std::string v = argv[i];
      if      (v == "int") o.clk = INTERNAL_CLOCK;
      else if (v == "ext") o.clk = EXTERNAL_CLOCK;
      else { std::cerr << "Bad --clk: " << v << "\n"; return false; }
    }
    else
    {
      std::cerr << "Unknown argument: " << a << "\n";
      usage(argv[0]);
      return false;
    }
  }
  return true;
}

} // namespace

int main(int argc, char** argv)
{
  CliOptions opt;
  if (!parse(argc, argv, opt)) return 1;

  std::cout << "Program for EVAL-AD5933\n";

  try
  {
    Ad5933 ad5933;
    ad5933.connect(VID, PID, opt.fwPath);
    std::cout << "Initialization completed.\n";
    sleep(1);

    UserParameters_st params{
      opt.refClock, opt.startFreq, opt.deltaFreq,
      opt.steps, opt.settle, DEFAULT_X1,
      opt.clk, opt.vrange, opt.pga,
      RES_ONLY, MID_POINT,
      opt.r1, 0.0, 0.0,
    };
    ad5933.setDeviceParameters(&params);

    Temperature_t temp = ad5933.readTemperature();
    std::cout << "Read temperature: " << temp << " C\n";
    sleep(1);

    if (!opt.quiet)
    {
      ad5933.setProgressCallback([](unsigned int i, const ImpedData_ct& s) {
        std::printf("[%3u] f=%.1f Hz  |Z_raw|=%.3f  phase=%.2f deg\n",
                    i, s.frequency, s.m, s.phase);
      });
    }

    ad5933.programDeviceRegisters();
    std::cout << "Calibration sweep...\n";
    ad5933.startSweep();
    std::cout << "Calibration...\n";
    ad5933.doCalibration();
    std::cout << "Gain factor: " << ad5933.getGainFactor() << "\n";

    std::cout << "Measurement sweep...\n";
    ad5933.programDeviceRegisters();
    ad5933.startSweep();
    ad5933.saveData(opt.output);
    std::cout << "Saved to " << opt.output << "\n";

    ad5933.deinit();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error: " << e.what() << "\n";
    return 2;
  }
  return 0;
}
