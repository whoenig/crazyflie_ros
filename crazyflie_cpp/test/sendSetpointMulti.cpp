#include <thread>
#include <iostream>

#include <crazyflie_cpp/Crazyflie.h>

int main()
{


  // launch two threads and set new setpoint at 100 Hz
  std::thread t1([] {
    Crazyflie cf1("radio://0/100/2M/E7E7E7E701"); // Instantiate first Crazyflie object
      // Send 0 a few times to make sure thrust lock is disabled
    for (size_t i = 0; i < 100; ++i) {
      cf1.sendSetpoint(0, 0, 0, 0);
    }
    while (true) {
      // Crazyflie cf1("radio://0/80/2M/E7E7E7E7E1");
      cf1.sendSetpoint(0, 0, 0, 10000); // send roll, pitch, yaw, and thrust
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // launch two threads and set new setpoint at 100 Hz
  std::thread t2([] {
    Crazyflie cf2("radio://0/100/2M/E7E7E7E702"); // Instantiate second Crazyflie object
      // Send 0 a few times to make sure thrust lock is disabled
    for (size_t i = 0; i < 100; ++i) {
      cf2.sendSetpoint(0, 0, 0, 0);
    }
    while (true) {
      // Crazyflie cf1("radio://0/80/2M/E7E7E7E7E1");
      cf2.sendSetpoint(0, 0, 0, 20000); // send roll, pitch, yaw, and thrust
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  t1.join();
  t2.join();

  return 0;
}
