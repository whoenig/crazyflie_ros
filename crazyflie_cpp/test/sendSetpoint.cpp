#include <thread>

#include <crazyflie_cpp/Crazyflie.h>

int main()
{
  Crazyflie cf1("radio://0/100/2M/E7E7E7E701");
  // Send 0 a few times to make sure thrust lock is disabled
  for (size_t i = 0; i < 100; ++i) {
    cf1.sendSetpoint(0, 0, 0, 0);
  }
  while (true) {
    cf1.sendSetpoint(0, 0, 0, 10000); // send roll, pitch, yaw, and thrust
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
