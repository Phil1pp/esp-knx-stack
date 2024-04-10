#include "libretiny_platform.h"

#ifdef LIBRETINY
#include <Arduino.h>
#include "knx/bits.h"
#include <StreamString.h>

#ifndef KNX_SERIAL
#define KNX_SERIAL Serial
#endif

LibretinyPlatform::LibretinyPlatform()
#ifndef KNX_NO_DEFAULT_UART
    : ArduinoPlatform(&KNX_SERIAL)
#endif
{
}

LibretinyPlatform::LibretinyPlatform( HardwareSerial* s) : ArduinoPlatform(s)
{
}

uint32_t LibretinyPlatform::currentIpAddress()
{
    return WiFi.localIP();
}

uint32_t LibretinyPlatform::currentSubnetMask()
{
    return WiFi.subnetMask();
}

uint32_t LibretinyPlatform::currentDefaultGateway()
{
    return WiFi.gatewayIP();
}

uint32_t LibretinyPlatform::uniqueSerialNumber()
{
    return lt_cpu_get_mac_id();
}

void LibretinyPlatform::restart()
{
    println("restart");
    lt_reboot();
}

void LibretinyPlatform::setupMultiCast(uint32_t addr, uint16_t port)
{
    _multicastAddr = htonl(addr);
    _multicastPort = port;
    IPAddress mcastaddr(_multicastAddr);
    StreamString mac, ip;
    mac.reserve(16);
    mcastaddr.printTo(mac);
    ip.reserve(16);
    WiFi.localIP().printTo(ip);

    KNX_DEBUG_SERIAL.printf("setup multicast addr: %s port: %d ip: %s\n", mac.c_str(), port,
        ip.c_str());
    uint8_t result = _udp.beginMulticast(WiFi.localIP(), port);
    KNX_DEBUG_SERIAL.printf("multicast setup result %d\n", result);
}

void LibretinyPlatform::closeMultiCast()
{
    _udp.stop();
}

bool LibretinyPlatform::sendBytesMultiCast(uint8_t * buffer, uint16_t len)
{
    //printHex("<- ",buffer, len);
    _udp.beginMulticastPacket();
    _udp.write(buffer, len);
    _udp.endPacket();
    return true;
}

int LibretinyPlatform::readBytesMultiCast(uint8_t * buffer, uint16_t maxLen)
{
    int len = _udp.parsePacket();
    if (len == 0)
        return 0;
    
    if (len > maxLen)
    {
        KNX_DEBUG_SERIAL.printf("udp buffer to small. was %d, needed %d\n", maxLen, len);
        fatalError();
    }

    _udp.read(buffer, len);
    //printHex("-> ", buffer, len);
    return len;
}

bool LibretinyPlatform::sendBytesUniCast(uint32_t addr, uint16_t port, uint8_t* buffer, uint16_t len)
{
    IPAddress ucastaddr(htonl(addr));
    println("sendBytesUniCast endPacket fail");
    if(_udp.beginPacket(ucastaddr, port) == 1) {
        _udp.write(buffer, len);
        if(_udp.endPacket() == 0) println("sendBytesUniCast endPacket fail");
    }
    else println("sendBytesUniCast beginPacket fail");
    return true;
}

#endif
