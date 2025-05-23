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
    case GET_RPID_ID:
      if (_callbacks[GET_RPID_ID] != NULL)
      {
        _callbacks[GET_RPID_ID](NULL);
      }      
      break;      
    case GET_PPID_ID:
      if (_callbacks[GET_PPID_ID] != NULL)
      {
        _callbacks[GET_PPID_ID](NULL);
      }      
      break;
    case GET_STATUS_ID:
      if (_callbacks[GET_STATUS_ID] != NULL)
      {
        _callbacks[GET_STATUS_ID](NULL);
      }
      break;
    case TEST_MOTORS_ID:
      if (len == 1 + sizeof(test_motors_msg_t) && _callbacks[TEST_MOTORS_ID] != NULL)
      {
        _callbacks[TEST_MOTORS_ID]((void*)pMessage);
      }
      break;
    case ARM_MOTORS_ID:
      if (len == 1 + sizeof(bool) && _callbacks[ARM_MOTORS_ID] != NULL)
      {
        _callbacks[ARM_MOTORS_ID]((void*)pMessage);
      }
      break;
    case SET_ROLL_PID_ID:
      if (len == 1 + sizeof(PidParamsVec) && _callbacks[SET_ROLL_PID_ID])
      {
        _callbacks[SET_ROLL_PID_ID]((void*)pMessage);
      }
      break;
    case SET_PITCH_PID_ID:
      if (len == 1 + sizeof(PidParamsVec) && _callbacks[SET_PITCH_PID_ID])
      {
        _callbacks[SET_PITCH_PID_ID]((void*)pMessage);
      }
      break;        
    default:
      break;
  }
}


void UDPServer::answerTo(uint8_t cmdId, uint8_t* pData, uint32_t len)
{
  general_msg_t answer;
  answer.msg_id = cmdId;
  memcpy(answer.payload, pData, len);
  _udp.broadcast((uint8_t*)&answer, 1 + len);
}


IPAddress UDPServer::localIp()
{
  return WiFi.localIP();
}
