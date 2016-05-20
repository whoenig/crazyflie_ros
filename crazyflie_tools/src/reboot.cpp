#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

enum Mode
{
  RebootToFirmware,
  RebootToBootloader,
};

std::istream& operator>>(std::istream& in, Mode& mode)
{
  std::string token;
  in >> token;
  std::cout << token << std::endl;
  if (token == "firmware")
    mode = RebootToFirmware;
  else if (token == "bootloader")
    mode = RebootToBootloader;
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
    ("mode", po::value<Mode>(&mode)->default_value(mode), "reboot mode {firmware,bootloader}")
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
    std::cout << mode << std::endl;
    Crazyflie cf(uri);
    switch (mode)
    {
      case RebootToFirmware:
        cf.reboot();
        break;
      case RebootToBootloader:
        cf.rebootToBootloader();
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
