#include <iostream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyradio.h>

int main(int argc, char **argv)
{

  std::string addressStr;
  std::string defaultAddressStr("0xE7E7E7E7E7");

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("address", po::value<std::string>(&addressStr)->default_value(defaultAddressStr), "device address")
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
    uint64_t address;
    std::stringstream sstr;
    sstr << std::hex << addressStr;
    sstr >> address;

    Crazyradio radio(0);
    radio.setAddress(address);

    for (uint8_t datarate = 0; datarate < 3; ++datarate) {
      radio.setDatarate((Crazyradio::Datarate)datarate);
      for (uint8_t channel = 0; channel <= 125; ++channel) {
        radio.setChannel(channel);

        uint8_t test[] = {0xFF};
        Crazyradio::Ack ack;
        radio.sendPacket(test, sizeof(test), ack);
        if (ack.ack) {
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
    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
