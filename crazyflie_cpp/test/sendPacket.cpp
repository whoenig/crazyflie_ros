#include <iostream>

#include <crazyflie_cpp/Crazyradio.h>

int main()
{
  Crazyradio radio(0); // Instantiate an object bound to the first Crazyflie found
  radio.setChannel(100); // Update the base frequency to 2500 MHz
  radio.setAddress(0xE7E7E7E701); // Set the address to send to
  radio.setDatarate(Crazyradio::Datarate_2MPS);
  // Send a packet
  uint8_t data[] = {0xCA, 0xFE};
  Crazyradio::Ack ack;
  radio.sendPacket(data, sizeof(data), ack);
  if (ack.ack) {
    std::cout << "Ack of size " << ack.size << " received!" << std::endl;
  } else {
    std::cout << "No Ack received!" << std::endl;
  }

  return 0;
}
