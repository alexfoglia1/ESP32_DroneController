#ifndef COMM_PROTO_H
#define COMM_PROTO_H

#include <stdint.h>

#define CTRL_ID          0x00
#define GET_ACCEL_ID     0x01
#define GET_GYRO_ID      0x02
#define GET_ATTITUDE_ID  0x03
#define GET_RPID_ID      0x04
#define GET_PPID_ID      0x05
#define GET_STATUS_ID    0x06
#define TEST_MOTORS_ID   0x07
#define ARM_MOTORS_ID    0x08
#define SET_ROLL_PID_ID  0x09
#define SET_PITCH_PID_ID 0x0A

#define MSGS_IN_SIZE    11
#define MAX_PAYLOAD_SIZE 32


typedef struct
{
  uint8_t msg_id;
  uint8_t payload[MAX_PAYLOAD_SIZE];
} general_msg_t;


typedef union
{
  struct
  {
    float x;
    float y;
    float z;
  } __attribute__((packed)) fields;
  float fvec[3];
  uint8_t bytes[12];
} Vec3D;

typedef union
{
  struct
  {
    float P;
    float I;
    float D;
    float U;
  } __attribute__((packed)) fields;
  float fvec[4];
  uint8_t bytes[16];
} PidVec;

typedef union
{
  struct
  {
    float Kp;
    float Ki;
    float Kd;
  } __attribute__((packed)) fields;
  float fvec[3];
  uint8_t bytes[12];
} PidParamsVec;

typedef union
{
    struct
    {
        uint8_t M1;
        uint8_t M2;
        uint8_t M3;
        uint8_t M4;
        uint8_t throttle_sp;
        float   roll_sp;
        float   pitch_sp;
    }__attribute__((packed)) fields;
    uint8_t bytes[13];
} status_msg_t;

typedef union
{
    struct
    {
        uint8_t  motors_flag;
        uint8_t  throttle;
    }__attribute__((packed)) fields;
    uint8_t bytes[2];
} test_motors_msg_t;


typedef struct
{
  uint8_t throttle;
  Vec3D   set_point;
} __attribute__((packed)) ctrl_msg_t;


#endif //COMM_PROTO_H
