#ifndef _NETCONFIG_H_
#define _NETCONFIG_H_
#include <ESP8266WiFi.h>
/**
 Important: rename this file to netconfig.h and change to your own parameters
*/

/*
TODO: make these parameters configurable
*/
const char* ssid = "SSID";  //  your network SSID (name)
const char* pass = "PASS";       // your network password

// set this to 0.0.0.0 for DHCP or provide IP for static address
IPAddress selfIp(192, 168, 1, 100);

// only need if device IP is static
IPAddress dns(192, 168, 1, 1);
IPAddress gw(192, 168, 1, 1);

// your PC's IP
IPAddress destIp(192, 168, 2, 5);
// UDP port for tracker software
uint16_t destPort = 5550;

#endif /* end of include guard: _NETCONFIG_H_ */
