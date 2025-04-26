#include "UDPServer.h"

#include <Arduino.h>
#include <stddef.h>

UDPServer::UDPServer()
{
  for (int i = 0; i < MSGS_IN_SIZE; i++)
  {
    _callbacks[i] = NULL;
  }
}


bool UDPServer::attachToWifi(const char* ssid, const char* password)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  return (WiFi.waitForConnectResult() == WL_CONNECTED);
}


void UDPServer::listen(short port)
{
  if (!_udp.listen(port))
  {
    Serial.println("Listen Failed");
  }
  else
  {
    _udp.onPacket([this](AsyncUDPPacket packet) { handleMessageRx(packet); });
  }
}


void UDPServer::addMessageCallback(int msgId, msg_callback_t callback)
{
  if (msgId < MSGS_IN_SIZE)
  {
    _callbacks[msgId] = callback;
  }
}


void UDPServer::handleMessageRx(AsyncUDPPacket packet)
{
  uint8_t* pData = packet.data();
  uint32_t len = packet.length();

  general_msg_t* pMessage = (general_msg_t*)pData;

  Serial.print("[INFO] <"); Serial.print(packet.remoteIP()); Serial.print(":"); Serial.print(packet.remotePort());
  Serial.print("> Received msgId("); Serial.print(pMessage->msg_id); Serial.print(") len("); Serial.print(len); Serial.println(")");

  switch (pMessage->msg_id)
  {
    case CTRL_ID:
      if (len == 1 + sizeof(ctrl_msg_t) && _callbacks[CTRL_ID] != NULL)
      {
        _callbacks[CTRL_ID]((void*)pMessage);
      }
      break;
    case GET_ACCEL_ID:
      if (_callbacks[GET_ACCEL_ID] != NULL)
      {
        _callbacks[GET_ACCEL_ID](NULL);
      }
      break;
    case GET_GYRO_ID:
      if (_callbacks[GET_GYRO_ID] != NULL)
      {
        _callbacks[GET_GYRO_ID](NULL);
      }    
      break;
    case GET_ATTITUDE_ID:
      if (_callbacks[GET_ATTITUDE_ID] != NULL)
      {
        _callbacks[GET_ATTITUDE_ID](NULL);
      }  
      break;
    case GET_PID_ID:
      if (_callbacks[GET_PID_ID] != NULL)
      {
        _callbacks[GET_PID_ID](NULL);
      }      
      break;
    default:
      break;
  }
}


void UDPServer::broadcast(uint8_t* pData, uint32_t len)
{
  _udp.broadcast(pData, len);
}


IPAddress UDPServer::localIp()
{
  return WiFi.localIP();
}
