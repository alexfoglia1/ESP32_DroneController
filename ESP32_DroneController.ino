#include "Maint.h"
#include "UDPServer.h"
#include "MPU6050.h"
#include "Wire.h"

#include <esp32-hal-ledc.h>
#include <stddef.h>
#include <string.h>

#define SDA_PIN 8
#define SCL_PIN 9
#define UDP_PORT 1234

#define IMU_UPDATE_MILLIS 10

#define MOTOR_1 5
#define MOTOR_2 6
#define MOTOR_3 20
#define MOTOR_4 21

const int MOTOR_PINS[4] = {MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4};

typedef struct
{
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int64_t update_t;
}__attribute__((packed)) mpu6050_data_t;

MPU6050 mpu6050;
mpu6050_data_t mpu6050_data;

typedef struct
{
  uint8_t motor_1_throttle;
  uint8_t motor_2_throttle;
  uint8_t motor_3_throttle;
  uint8_t motor_4_throttle;
} motor_throttle_t;

uint8_t throttle_setpoint;
motor_throttle_t throttle_command;
motor_throttle_t throttle_status;

Maintenance* maint;
UDPServer* udpServer;

char wifi_ssid[256];
char wifi_pwd[256];


// ----------------- MAINTENANCE CALLBACKS ----------------- 
void onRxSSID(void* rxSSID)
{
  const char* ssid = (const char*) rxSSID;
  memcpy(wifi_ssid, ssid, strlen(ssid));

  if (strlen(wifi_pwd))
  {
    if (udpServer->attachToWifi(wifi_ssid, wifi_pwd))
    {
      Serial.print("[OK] <"); Serial.print(udpServer->localIp()); Serial.print("> Attached to WIFI ");

      udpServer->listen(UDP_PORT);
    }
    else
    {
      Serial.print("[NOK] Cannot connect to WIFI ");
    }

    Serial.print(wifi_ssid); Serial.print(" using password "); Serial.println(wifi_pwd);
  }
}


void onRxPWD(void* rxPWD)
{
  const char* pwd = (const char*) rxPWD;
  memcpy(wifi_pwd, pwd, strlen(pwd));

  if (strlen(wifi_ssid))
  {
    if (udpServer->attachToWifi(wifi_ssid, wifi_pwd))
    {   
      Serial.print("[OK] <"); Serial.print(udpServer->localIp()); Serial.print("> Attached to WIFI ");

      udpServer->listen(UDP_PORT);   
    }
    else
    {
      Serial.print("[NOK] Cannot connect to WIFI ");
    }

    Serial.print(wifi_ssid); Serial.print(" using password "); Serial.println(wifi_pwd);
  }
}
// ----------------- MAINTENANCE CALLBACKS ----------------- 


// ----------------- UDP SERVER CALLBACKS ------------------ 
void onGetGyro(void* data)
{
  Vec3D gyro;
  gyro.fields.x = mpu6050_data.gyro_x / 131.0f;
  gyro.fields.y = mpu6050_data.gyro_y / 131.0f;
  gyro.fields.z = mpu6050_data.gyro_z / 131.0f;

  udpServer->broadcast(gyro.bytes, sizeof(gyro.bytes));
}


void onGetAccel(void* data)
{
  Vec3D accel;
  accel.fields.x = mpu6050_data.accel_x / 16384.0f;
  accel.fields.y = mpu6050_data.accel_y / 16384.0f;
  accel.fields.z = mpu6050_data.accel_z / 16384.0f;

  udpServer->broadcast(accel.bytes, sizeof(accel.bytes));
}


void onControlCommand(void* data)
{
  general_msg_t* msgIn = (general_msg_t*) data;
  ctrl_msg_t* ctrlCmdIn = (ctrl_msg_t*) &msgIn->payload[0];

  throttle_setpoint = ctrlCmdIn->throttle;
}
// ----------------- UDP SERVER CALLBACKS ------------------ 


// ----------------- PID CONTROLLER -----------------
// TODO: fare un vero pid
void pid_controller(uint8_t throttle_sp, uint8_t roll_sp, uint8_t roll, uint8_t pitch_sp, uint8_t pitch)
{
  throttle_command.motor_1_throttle = throttle_sp; // + pid...
  throttle_command.motor_2_throttle = throttle_sp; // + pid...
  throttle_command.motor_3_throttle = throttle_sp; // + pid...
  throttle_command.motor_4_throttle = throttle_sp; // + pid...
}
// ----------------- PID CONTROLLER -----------------


void setup()
{
// -------- MOTORS INIT ----------
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(MOTOR_3, OUTPUT);
  pinMode(MOTOR_4, OUTPUT);

  digitalWrite(MOTOR_1, LOW);
  digitalWrite(MOTOR_2, LOW);
  digitalWrite(MOTOR_3, LOW);
  digitalWrite(MOTOR_4, LOW);

  const int freq = 10000;   // 10 kHz
  const int resolution = 8; // 8 bit (0-255)
  for (int i = 0; i < 4; i++)
  {
    ledcAttach(MOTOR_PINS[i], freq, resolution);
  }

  throttle_command.motor_1_throttle = 0;
  throttle_command.motor_2_throttle = 0;
  throttle_command.motor_3_throttle = 0;
  throttle_command.motor_4_throttle = 0;
  throttle_status.motor_1_throttle  = 0;
  throttle_status.motor_2_throttle  = 0;
  throttle_status.motor_3_throttle  = 0;
  throttle_status.motor_4_throttle  = 0;
  throttle_setpoint = 0;
// -------- MOTORS INIT ----------

  delay(1000);

// -------- MPU6050 INIT --------
  mpu6050_data.gyro_x = 0x00;
  mpu6050_data.gyro_y = 0x00;
  mpu6050_data.gyro_z = 0x00;
  mpu6050_data.accel_x = 0x00;
  mpu6050_data.accel_y = 0x00;
  mpu6050_data.accel_z = 0x00;
  mpu6050_data.update_t = -1;
  
  //Wire.begin(SDA_PIN, SCL_PIN);
  //mpu6050.initialize();
// -------- MPU6050 INIT --------

// -------- MAINTENANCE INIT --------
  maint = new Maintenance(115200);

  maint->addCommandCallback(SET_SSID_CMD_ID, onRxSSID);
  maint->addCommandCallback(SET_PWD_CMD_ID, onRxPWD);
// -------- MAINTENANCE INIT --------

// -------- UDP SERVER INIT --------
  udpServer = new UDPServer();
  
  udpServer->addMessageCallback(GET_ACCEL_ID, onGetAccel);
  udpServer->addMessageCallback(GET_GYRO_ID,  onGetGyro);
  udpServer->addMessageCallback(CTRL_ID, onControlCommand);
  
  memset(wifi_ssid, 0x00, sizeof(wifi_ssid));
  memset(wifi_pwd, 0x00, sizeof(wifi_pwd));
// -------- UDP SERVER INIT --------  
}


void loop()
{
  int64_t cur_t_micros = micros();
// ------- MOTORS UPDATE -------
  pid_controller(throttle_setpoint, 0, 0, 0, 0);

  // TODO: lega a comando ARM/DISARM
  //digitalWrite(N_SLEEP, throttle_setpoint > 0 ? HIGH : LOW);
  // TODO: fare per tutti i motori
  //int dir = throttle_command.motor_2_throttle > throttle_status.motor_2_throttle ? 1 : -1;
  //if (dir > 0)
  //{
  //  for (int throttle = throttle_status.motor_2_throttle; throttle <= throttle_command.motor_2_throttle; throttle += 1)
  //  {
  //    ledcWrite(MOTOR_PINS[1], throttle);
  //    delay(1);
  //  }
  //}
  //else
  //{
  //  ledcWrite(MOTOR_PINS[1], throttle_command.motor_2_throttle);
  //}

  ledcWrite(MOTOR_1, throttle_command.motor_1_throttle);
  ledcWrite(MOTOR_2, throttle_command.motor_2_throttle);
  ledcWrite(MOTOR_3, throttle_command.motor_3_throttle);
  ledcWrite(MOTOR_4, throttle_command.motor_4_throttle);

  throttle_status.motor_1_throttle = throttle_command.motor_1_throttle;
  throttle_status.motor_2_throttle = throttle_command.motor_2_throttle;
  throttle_status.motor_3_throttle = throttle_command.motor_3_throttle;
  throttle_status.motor_4_throttle = throttle_command.motor_4_throttle;
// ------- MOTORS UPDATE -------

// ------- IMU UPDATE -------
  if (mpu6050_data.update_t < 0 || (cur_t_micros - mpu6050_data.update_t) > IMU_UPDATE_MILLIS * 1000)
  {
    mpu6050_data.update_t = cur_t_micros;
    mpu6050.getMotion6(&mpu6050_data.accel_x,
                      &mpu6050_data.accel_y,
                      &mpu6050_data.accel_z,
                      &mpu6050_data.gyro_x,
                      &mpu6050_data.gyro_y,
                      &mpu6050_data.gyro_z);
  }
// ------- IMU UPDATE -------

// ------- MAINTENANCE UPDATE -------
  maint->update();
// ------- MAINTENANCE UPDATE -------
}
