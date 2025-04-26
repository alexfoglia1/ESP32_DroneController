#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include "WiFi.h"
#include "AsyncUDP.h"
#include "CommProto.h"

typedef void (*msg_callback_t)(void*);

class UDPServer
{
  public:
    UDPServer();

    bool attachToWifi(const char* ssid, const char* password);
    void listen(short port);
    void addMessageCallback(int msgId, msg_callback_t callback);
    void answerTo(uint8_t msgId, uint8_t* pData, uint32_t len);
    IPAddress localIp();

  private:
    void handleMessageRx(AsyncUDPPacket packet);

    AsyncUDP _udp;
    msg_callback_t _callbacks[MSGS_IN_SIZE];
    
};


#endif // UDP_SERVER_H
