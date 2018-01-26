#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>
#include <crazyflie_cpp/CrazyflieUSB.h>


int main(int argc, char **argv)
{

  std::string addressStr;
  std::string defaultAddressStr("0xE7E7E7E7E7");
  bool verbose = false;

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("address", po::value<std::string>(&addressStr)->default_value(defaultAddressStr), "device address")
    ("verbose,v", "verbose output")
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
    verbose = vm.count("verbose");
  }
  catch(po::error& e)
  {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  try
  {
    uint32_t numCrazyradios = Crazyradio::numDevices();
    if (numCrazyradios > 0) {
      uint64_t address;
      std::stringstream sstr;
      sstr << std::hex << addressStr;
      sstr >> address;

      Crazyradio radio(0);
      if (verbose) {
        std::cout << "Found Crazyradio with version " << radio.version() << std::endl;
      }
      radio.setAddress(address);
      size_t numCFsFound = 0;
      for (uint8_t datarate = 0; datarate < 3; ++datarate) {
        radio.setDatarate((Crazyradio::Datarate)datarate);
        for (uint8_t channel = 0; channel <= 125; ++channel) {
          radio.setChannel(channel);

          uint8_t test[] = {0xFF};
          Crazyradio::Ack ack;
          radio.sendPacket(test, sizeof(test), ack);
          if (ack.ack) {
            ++numCFsFound;
            std::cout << "radio://0/" << (uint32_t)channel << "/";
            switch(datarate) {
            case Crazyradio::Datarate_250KPS:
              std::cout << "250K";
              break;
            case Crazyradio::Datarate_1MPS:
              std::cout << "1M";
              break;
            case Crazyradio::Datarate_2MPS:
              std::cout << "2M";
              break;
            }

            if (defaultAddressStr != addressStr) {
              std::cout << "/" << addressStr.substr(2);
            }
            std::cout << std::endl;
          }
        }
      }

      if (   numCFsFound == 0
          && verbose) {
        std::cout << "No Crazyflie found. Did you specify the correct address?" << std::endl;
      }
    } else if (verbose) {
      std::cout << "No Crazyradio found." << std::endl;
    }

    uint32_t numCFoverUSB = CrazyflieUSB::numDevices();
    if (numCFoverUSB > 0) {
      CrazyflieUSB cfusb(0);
      if (verbose) {
        std::cout << "Found Crazyflie connected via USB cable with version " << cfusb.version() << std::endl;
      }

      for (uint32_t i = 0; i < numCFoverUSB; ++i) {
        std::cout << "usb://" << i << std::endl;
      }

    } else if (verbose) {
      std::cout << "No Crazyflie connected via USB cable found." << std::endl;
    }
    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
