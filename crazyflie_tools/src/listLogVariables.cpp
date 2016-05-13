#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

int main(int argc, char **argv)
{

  std::string uri;
  std::string defaultUri("radio://0/80/2M/E7E7E7E7E7");

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
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
    cf.requestLogToc();

     std::for_each(cf.logVariablesBegin(), cf.logVariablesEnd(),
      [](const Crazyflie::LogTocEntry& entry)
      {
        std::cout << entry.group << "." << entry.name << " (";
        switch (entry.type) {
        case Crazyflie::LogTypeUint8:
          std::cout << "uint8";
          break;
        case Crazyflie::LogTypeInt8:
          std::cout << "int8";
          break;
        case Crazyflie::LogTypeUint16:
          std::cout << "uint16";
          break;
        case Crazyflie::LogTypeInt16:
          std::cout << "int16";
          break;
        case Crazyflie::LogTypeUint32:
          std::cout << "uint32";
          break;
        case Crazyflie::LogTypeInt32:
          std::cout << "int32";
          break;
        case Crazyflie::LogTypeFloat:
          std::cout << "float";
          break;
        case Crazyflie::LogTypeFP16:
          std::cout << "fp16";
          break;
        }
        std::cout << ")";
        std::cout << std::endl;
      }
    );

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
