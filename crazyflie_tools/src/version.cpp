#include <iostream>
#include <chrono>
#include <thread>

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

    uint32_t revision0 = 0;
    uint16_t revision1 = 0;
    uint32_t revchanged0 = 0;
    uint16_t revchanged1 = 0;
    uint8_t modified = 1;

    const Crazyflie::ParamTocEntry* entry = cf.getParamTocEntry("firmware", "revision0");
    if (entry) {
      revision0 = cf.getParam<uint32_t>(entry->id);
    }

    entry = cf.getParamTocEntry("firmware", "revision1");
    if (entry) {
      revision1 = cf.getParam<uint16_t>(entry->id);
    }

    entry = cf.getParamTocEntry("firmware", "modified");
    if (entry) {
      modified = cf.getParam<uint8_t>(entry->id);
    }

    entry = cf.getParamTocEntry("firmware", "revchanged0");
    if (entry) {
      revchanged0 = cf.getParam<uint32_t>(entry->id);
    }

    entry = cf.getParamTocEntry("firmware", "revchanged1");
    if (entry) {
      revchanged1 = cf.getParam<uint16_t>(entry->id);
    }

    uint64_t revision = ((uint64_t)revision0 << 16) | revision1;
    uint64_t revChanged = ((uint64_t)revchanged0 << 16) | revchanged1;

    std::cout << std::hex << revision << "," << (bool)modified << "," << revChanged << std::endl;

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
