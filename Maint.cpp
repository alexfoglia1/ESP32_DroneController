#include "Maint.h"

#include <Arduino.h>
#include <string.h>
#include <stddef.h>


Maintenance::Maintenance(int baud)
{
  Serial.begin(baud);
  
  _rxStatus = WAIT_CMD_STR;
  _rxBytes = 0;
  _expectedLen = 0;

  memset(_rxBuffer, 0x00, sizeof(_rxBuffer));
  for (int i = 0; i < N_MAINT_COMMANDS; i++)
  {
    _callbacks[i] = 0;
  }
  _selectedCallback = 0;


  Serial.println("[OK] Maintenance Initialized");
}


void Maintenance::update()
{
  while (Serial.available())
  {
    uint8_t rxByte = Serial.read() & 0xFF;
    updateFsm(rxByte);
  }
}


void Maintenance::addCommandCallback(uint8_t command, maint_cmd_callback_t callback)
{
  if (command < N_MAINT_COMMANDS)
  {
    _callbacks[command] = callback;
  }
}


void Maintenance::updateFsm(uint8_t rxByte)
{
  _rxBuffer[_rxBytes] = rxByte;
  _rxBytes += 1;

  switch (_rxStatus)
  {
    case WAIT_CMD_STR:  checkCmdStr((maint_msg_t*) _rxBuffer); break;
    case WAIT_PAYLOAD:  checkPayload((maint_msg_t*) _rxBuffer, rxByte); break;
    default: break;
  }
}


uint8_t Maintenance::toCmdId(maint_msg_t* rxMsg)
{
  if (strncmp(rxMsg->cmd_str, SET_SSID_CMD_STR, strlen(SET_SSID_CMD_STR)) == 0)
  {
    return SET_SSID_CMD_ID;
  }
  if (strncmp(rxMsg->cmd_str, SET_PWD_CMD_STR, strlen(SET_PWD_CMD_STR)) == 0)
  {
    return SET_PWD_CMD_ID;
  }
  if (strncmp(rxMsg->cmd_str, GET_PWD_CMD_STR, strlen(GET_PWD_CMD_STR)) == 0)
  {
    return GET_PWD_CMD_ID;
  }
  if (strncmp(rxMsg->cmd_str, GET_SSID_CMD_STR, strlen(GET_SSID_CMD_STR)) == 0)
  {
    return GET_SSID_CMD_ID;
  }
  if (strncmp(rxMsg->cmd_str, GET_IP_CMD_STR, strlen(GET_IP_CMD_STR)) == 0)
  {
    return GET_IP_CMD_ID;
  }
  if (strncmp(rxMsg->cmd_str, GET_LOOP_T_CMD_STR, strlen(GET_LOOP_T_CMD_STR)) == 0)
  {
    return GET_LOOP_T_CMD_ID;
  }
  if (strncmp(rxMsg->cmd_str, IMU_CALIB_CMD_STR, strlen(IMU_CALIB_CMD_STR)) == 0)
  {
    return IMU_CALIB_CMD_ID;
  }
  
  return INCOMPLETE_CMD_ID;
}


void Maintenance::checkCmdStr(maint_msg_t* rxMsg)
{
  uint8_t cmdId = toCmdId(rxMsg);

  switch (cmdId)
  {
    case SET_SSID_CMD_ID:
      _expectedLen = MAX_PAYLOAD_SIZE_MAINT;
      _selectedCallback = SET_SSID_CMD_ID;
      Serial.println("[OK] <SET_SSID>");
      _rxStatus = WAIT_PAYLOAD;

    break;
    case SET_PWD_CMD_ID:
      _expectedLen = MAX_PAYLOAD_SIZE_MAINT;
      _selectedCallback = SET_PWD_CMD_ID;
      Serial.println("[OK] <SET_PWD>");
      _rxStatus = WAIT_PAYLOAD;
    break;
    case GET_SSID_CMD_ID:
      _expectedLen = 1;
      _selectedCallback = GET_SSID_CMD_ID;
      Serial.println("[OK] <GET_SSID>");
      _rxStatus = WAIT_PAYLOAD;
    break;
    case GET_PWD_CMD_ID:
      _expectedLen = 1;
      _selectedCallback = GET_PWD_CMD_ID;
      Serial.println("[OK] <GET_PWD>");
      _rxStatus = WAIT_PAYLOAD;
    break;
    case GET_IP_CMD_ID:
      _expectedLen = 1;
      _selectedCallback = GET_IP_CMD_ID;
      Serial.println("[OK] <GET_IP>");
      _rxStatus = WAIT_PAYLOAD;
    break;
    case GET_LOOP_T_CMD_ID:
      _expectedLen = 1;
      _selectedCallback = GET_LOOP_T_CMD_ID;
      Serial.println("[OK] <GET_LOOP_T>");
      _rxStatus = WAIT_PAYLOAD;
    break;
    case IMU_CALIB_CMD_ID:
      _expectedLen = 4;
      _selectedCallback = IMU_CALIB_CMD_ID;
      Serial.println("[OK] <IMU_CALIB>");
      _rxStatus = WAIT_PAYLOAD;
    break;        
    case INCOMPLETE_CMD_ID:
      if (_rxBytes < sizeof(rxMsg->cmd_str))
      {
        break;
      }
    default:
      Serial.println("[NOK] <UNKNOWN COMMAND>");
      _rxStatus = WAIT_PAYLOAD; // Mi serve perchè devo attendere il fine comando
                                // altrimenti entro nel loop in cui accodo byte di comandi sbagliati come inizio dei successivi
      _selectedCallback = 0xFF;
      _expectedLen = 0;
    break;
  }
}


void Maintenance::checkPayload(maint_msg_t* rxMsg, uint8_t lastByte)
{
  if (_rxBytes == _expectedLen || lastByte == '\0')
  {
    if (_selectedCallback < N_MAINT_COMMANDS && _callbacks[_selectedCallback] != NULL)
    {
      Serial.print("[OK] <");
      Serial.print(_selectedCallback); Serial.print("> "); Serial.println((const char*)rxMsg->payload);   
       
      _callbacks[_selectedCallback](rxMsg->payload);
    }
    else
    {
      Serial.print("[NOK] <");
      Serial.print(_selectedCallback); Serial.print("> "); Serial.println((const char*)rxMsg->payload);    
    }

    _rxStatus = WAIT_CMD_STR;
    _rxBytes = 0;
    _expectedLen = 0;
    memset(_rxBuffer, 0x00, sizeof(_rxBuffer));
  }
}
