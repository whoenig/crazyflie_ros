#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

std::istream& operator>>(std::istream& in, Crazyflie::BootloaderTarget& target)
{
  std::string token;
  in >> token;
  if (token == "stm32")
    target = Crazyflie::TargetSTM32;
  else if (token == "nrf51")
    target = Crazyflie::TargetNRF51;
  else throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value);
  return in;
}

enum Mode
{
  FlashAndVerify,
  FlashOnly,
  VerifyOnly,
};

std::istream& operator>>(std::istream& in, Mode& mode)
{
  std::string token;
  in >> token;
  if (token == "flashAndVerify")
    mode = FlashAndVerify;
  else if (token == "flashOnly")
    mode = FlashOnly;
  else if (token == "verifyOnly")
    mode = VerifyOnly;
  else throw boost::program_options::validation_error(boost::program_options::validation_error::invalid_option_value);
  return in;
}

int main(int argc, char **argv)
{

  std::string fileName;
  std::string uri;
  std::string defaultUri("radio://0/0/2M");
  Crazyflie::BootloaderTarget target;
  Mode mode = FlashAndVerify;

  namespace po = boost::program_options;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("target", po::value<Crazyflie::BootloaderTarget>(&target)->required(), "target {stm32,nrf51}")
    ("filename", po::value<std::string>(&fileName)->required(), "file to flash")
    ("uri", po::value<std::string>(&uri)->default_value(defaultUri), "unique ressource identifier")
    ("mode", po::value<Mode>(&mode)->default_value(mode), "mode {default=flashAndVerify, flashOnly, verifyOnly}")
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
    bool success = true;
    if (uri != defaultUri) {
      // std::cout << "Reboot to Bootloader...";
      Crazyflie cf(uri);
      uint64_t address = cf.rebootToBootloader();
      // std::cout << "...Done" << std::endl;

      char addr[17];
      std::sprintf(addr, "%lx", address);
      defaultUri += "/" + std::string(addr);
    }

    std::ifstream stream(fileName.c_str(), std::ios::binary);
    std::vector<uint8_t> targetData((
      std::istreambuf_iterator<char>(stream)),
      (std::istreambuf_iterator<char>()));

    Crazyflie cf(defaultUri);

    if (mode == FlashAndVerify || mode == FlashOnly) {
      // std::cout << "Flashing " << targetData.size() / 1024 << " kB" << std::endl;
      cf.writeFlash(target, targetData);
    }
    if (mode == FlashAndVerify || mode == VerifyOnly) {
      // std::cout << "Reading " << targetData.size() / 1024 << " kB" << std::endl;
      std::vector<uint8_t> currentData;
      cf.readFlash(target, targetData.size(), currentData);
      std::ofstream dbg("data.bin", std::ios::binary);
      dbg.write((char*)currentData.data(), currentData.size());
      if (memcmp(targetData.data(), currentData.data(), targetData.size()) == 0) {
        // std::cout << "Verification successful!" << std::endl;
      } else {
        std::cout << "Verification NOT successful!" << std::endl;
        success = false;
      }
    }

    // std::cout << "Reboot to firmware" << std::endl;
    cf.reboot();

    if (success) {
      return 0;
    }
    return 1;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
