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
    cf.requestParamToc();

     std::for_each(cf.paramsBegin(), cf.paramsEnd(),
      [](const Crazyflie::ParamTocEntry& entry)
      {
        std::cout << entry.group << "." << entry.name << " (";
        switch (entry.type) {
        case Crazyflie::ParamTypeUint8:
          std::cout << "uint8";
          break;
        case Crazyflie::ParamTypeInt8:
          std::cout << "int8";
          break;
        case Crazyflie::ParamTypeUint16:
          std::cout << "uint16";
          break;
        case Crazyflie::ParamTypeInt16:
          std::cout << "int16";
          break;
        case Crazyflie::ParamTypeUint32:
          std::cout << "uint32";
          break;
        case Crazyflie::ParamTypeInt32:
          std::cout << "int32";
          break;
        case Crazyflie::ParamTypeFloat:
          std::cout << "float";
          break;
        }
        if (entry.readonly) {
          std::cout << ", readonly";
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
