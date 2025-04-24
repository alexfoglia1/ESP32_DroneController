#include "UDPServer.h"


UDPServer::UDPServer(const char* ssid, const char* password)
{
  callbacks = {NULL, NULL, NULL, NULL, NULL};

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("WiFi Failed");
  }
}


void UDPServer::listen(short port)
{
  if (!udp.listen(port))
  {
    Serial.println("Listen Failed");
  }
  else
  {
    udp.onPacket([](AsyncUDPPacket packet) { handleMsgRx(packet); });
  }
}

void UDPServer::handleMsgRx(AsyncUDPPacket packet)
{
  uint8_t* pData = packet.data();
  uint32_t len = packet.length();

  general_msg_t* pMessage = (general_msg_t*)pData;

  switch (pMessage->msg_id)
  {
    case CTRL_ID:
      if (len == sizeof(ctrl_msg_t) && callbacks[CTRL_ID] != NULL)
      {
        callbacks[CTRL_ID]((void*)pMessage);
      }
      break;
    case GET_ACCEL_ID:
      
      break;
    case GET_GYRO_ID:
      break;
    case GET_ATTITUDE_ID:
      break;
    case GET_PID_ID:
      break;
    default:
      break;
  }

}
