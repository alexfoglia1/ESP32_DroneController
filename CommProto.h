#ifndef COMM_PROTO_H
#define COMM_PROTO_H

#include <stdint.h>

#define CTRL_ID         0x00
#define GET_ACCEL_ID    0x01
#define GET_GYRO_ID     0x02
#define GET_ATTITUDE_ID 0x03
#define GET_PID_ID      0x04

#define MSGS_IN_SIZE    5
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

typedef struct
{
  uint8_t throttle;
  Vec3D   set_point;
} __attribute__((packed)) ctrl_msg_t;


#endif //COMM_PROTO_H
