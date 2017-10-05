#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

enum Mode
{
  RebootToFirmware,
  RebootToBootloader,
  SysOff,
  SysOn,
  AllOff,
};

std::istream& operator>>(std::istream& in, Mode& mode)
{
  std::string token;
  in >> token;
  if (token == "firmware")
    mode = RebootToFirmware;
  else if (token == "bootloader")
    mode = RebootToBootloader;
  else if (token == "sysoff")
    mode = SysOff;
  else if (token == "syson")
    mode = SysOn;
  else if (token == "alloff")
    mode = AllOff;
  else throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value);
  return in;
}

int main(int argc, char **argv)
{

  std::string uri;
  std::string defaultUri("radio://0/80/2M/E7E7E7E7E7");
  Mode mode = RebootToFirmware;

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
    ("mode", po::value<Mode>(&mode)->default_value(mode), "reboot mode {firmware,bootloader,sysoff,syson,alloff}")
  ;

  try
  {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  try
  {
    Crazyflie cf(uri);
    switch (mode)
    {
      case RebootToFirmware:
        cf.reboot();
        break;
      case RebootToBootloader:
        cf.rebootToBootloader();
        break;
      case SysOff:
        cf.sysoff();
        break;
      case SysOn:
        cf.syson();
        break;
      case AllOff:
        cf.alloff();
        break;
    }

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
