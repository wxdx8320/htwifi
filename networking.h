#ifndef HTWIFI_NETWORKING_H
#define HTWIFI_NETWORKING_H
#include <IPAddress.h>
#include <stdint.h>

void InitWifi(void);
bool sendBytes(unsigned char *buffer, unsigned int length);
void configNetwork(const char * board, const char *host, unsigned int port);
void configWiFi(char* ssid, char *password);
unsigned int receiveBytes(uint8_t *buf, uint16_t length);

#endif /* end of include guard: HTWIFI_ */
