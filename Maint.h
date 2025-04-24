#ifndef MAINT_H
#define MAINT_H

#include <stdint.h>

#define MAX_PAYLOAD_SIZE_MAINT 512

#define SET_SSID_CMD_ID    0x00
#define SET_PWD_CMD_ID     0x01
#define INCOMPLETE_CMD_ID  0x02

#define SET_SSID_CMD_STR "SSID: "
#define SET_PWD_CMD_STR  "PSWD: "

#define N_MAINT_COMMANDS 2


typedef void (*maint_cmd_callback_t)(void*);

typedef struct
{
  char cmd_str[6];
  char payload[MAX_PAYLOAD_SIZE_MAINT];
}__attribute__((packed)) maint_msg_t;


typedef enum
{
  WAIT_CMD_STR,
  WAIT_PAYLOAD
} rx_status_t;


class Maintenance
{
  public:
    Maintenance(int baud);

    void update();
    void addCommandCallback(uint8_t command, maint_cmd_callback_t callback);
    
  private:
    uint32_t    _rxBytes;
    uint32_t    _expectedLen;
    rx_status_t _rxStatus;

    maint_cmd_callback_t _callbacks[N_MAINT_COMMANDS];
    uint8_t _selectedCallback;
    
    uint8_t _rxBuffer[2 * MAX_PAYLOAD_SIZE_MAINT];

    void updateFsm(uint8_t rxByte);
    uint8_t toCmdId(maint_msg_t* rxMsg);
    void checkCmdStr(maint_msg_t* rxMsg);
    void checkPayload(maint_msg_t* rxMsg, uint8_t lastByte);

};


#endif // MAINT_H
