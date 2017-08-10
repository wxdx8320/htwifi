#include "networking.h"
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include "netconfig.h"

int status = WL_IDLE_STATUS;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
//  Serial.print("ESP8266 ID#");
//  Serial.println(system_get_chip_id());

}

void InitWifi() {
  // setting up Station AP
  WiFi.begin(ssid, pass);
  if(selfIp != IPAddress(0,0,0,0)) {
    WiFi.config(selfIp, dns, gw);
  }

  // Wait for connect to AP
  Serial.print("[Connecting]");
  Serial.print(ssid);
  int tries=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    if (tries > 30){
      break;
    }
  }
  Serial.println(tries);


  printWifiStatus();

  if(tries <= 30)
    Serial.println("Connected to wifi");
  else
    Serial.println("Unable to connect to WiFi");
//  Serial.print("Udp server started at port ");
//  Serial.println(localPort);
//  Udp.begin(localPort);
}

bool sendBytes(unsigned char *buffer, unsigned int length) {
  int result;
  result = udp.beginPacket(destIp, destPort);
  if(result != 1)
    return false;
  udp.write(buffer, length);
  result = udp.endPacket();
  if(result != 1)
    return false;
  return true;
}

unsigned int receiveBytes(uint8_t *buf, uint16_t length) {
  unsigned int avail = udp.parsePacket();
  if(avail == 0) {
    return 0;
  } else if(avail > length) {
    avail=length;
  }
  avail = udp.read(buf, avail);
  return avail;
}
void configNetwork(const char *board, const char *hostIp, unsigned int hostPort) {
  // FIXME: make actual config
}

void configWiFi(char* ssid, char *password) {
  // FIXME: make actual config

}
