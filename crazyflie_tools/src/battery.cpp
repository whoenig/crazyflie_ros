#include <iostream>
#include <chrono>
#include <thread>

#include <boost/program_options.hpp>
#include <crazyflie_cpp/Crazyflie.h>

struct log {
  float pm_vbat;
  // float chargeCurrent;
} __attribute__((packed));

volatile bool g_done = false;

void onLogData(uint32_t time_in_ms, struct log* data)
{
  std::cout << data->pm_vbat << std::endl;
  g_done = true;
}

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
    std::cout << cf.vbat() << std::endl;
    // cf.logReset();
    // cf.requestLogToc();

    // std::unique_ptr<LogBlock<struct log> > logBlock;
    // std::function<void(uint32_t, struct log*)> cb = std::bind(&onLogData, std::placeholders::_1, std::placeholders::_2);

    // logBlock.reset(new LogBlock<struct log>(
    //   &cf,{
    //     {"pm", "vbat"},
    //     // {"pm", "chargeCurrent"}
    //   }, cb));
    // logBlock->start(10); // 100ms

    // while (!g_done) {
    //   cf.sendPing();
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    return 0;
  }
  catch(std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
