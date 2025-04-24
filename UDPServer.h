#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include "WiFi.h"
#include "AsyncUDP.h"
#include "CommProto.h"

typedef void (*msg_callback_t)(void*);

class UDPServer
{
  public:
    UDPServer(const char* ssid, const char* password);
    void listen(short port);
    void addMessageCallback();

  private:
    void handleMessageRx(AsyncUDPPacket packet);

    AsyncUDP udp;
    msg_callback_t callbacks[MSGS_IN_SIZE];
    
};


#endif // UDP_SERVER_H