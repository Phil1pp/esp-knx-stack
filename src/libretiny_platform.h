#ifdef LIBRETINY

#include "arduino_platform.h"
#include <WiFi.h>
#include <WiFiUdp.h>

class LibretinyPlatform : public ArduinoPlatform
{
  public:
    LibretinyPlatform();
    LibretinyPlatform(HardwareSerial* s);

    // ip stuff
    uint32_t currentIpAddress() override;
    uint32_t currentSubnetMask() override;
    uint32_t currentDefaultGateway() override;

    // unique serial number
    uint32_t uniqueSerialNumber() override;

    // basic stuff
    void restart();

    //multicast
    void setupMultiCast(uint32_t addr, uint16_t port) override;
    void closeMultiCast() override;
    bool sendBytesMultiCast(uint8_t* buffer, uint16_t len) override;
    int readBytesMultiCast(uint8_t* buffer, uint16_t maxLen) override;
   
    //unicast 
    bool sendBytesUniCast(uint32_t addr, uint16_t port, uint8_t* buffer, uint16_t len) override;
    
private:
    WiFiUDP _udp;
    uint32_t _multicastAddr;
    uint16_t _multicastPort;
};

#endif
