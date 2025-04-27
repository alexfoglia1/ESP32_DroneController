#include "Maint.h"
#include "UDPServer.h"
#include "RPI_MPU6050.h"
#include "Wire.h"
#include "ComplementaryFilter.h"

#include <esp32-hal-ledc.h>
#include <stddef.h>
#include <string.h>
#include <Preferences.h>

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
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accel_x;
  float accel_y;
  float accel_z;
  int64_t update_t;
}__attribute__((packed)) mpu6050_data_t;

RPI_MPU6050 mpu6050;
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
Preferences preferences;
ComplementaryFilter attitudeFilter;

void tryWifiConnection(const char* ssid, const char* pwd)
{
  if (strlen(ssid) && strlen(pwd))
  {
    if (udpServer->attachToWifi(ssid, pwd))
    {   
      Serial.print("[OK] <"); Serial.print(udpServer->localIp()); Serial.print("> Attached to WIFI ");
            
      preferences.begin("storage");
      preferences.putString("ssid", ssid);
      preferences.putString("pwd", pwd);
      preferences.end();

      udpServer->listen(UDP_PORT);
    }
    else
    {
      Serial.print("[NOK] Cannot connect to WIFI ");
    }

    Serial.print(ssid); Serial.print(" using password "); Serial.println(pwd);
  }
}


void imuUpdate()
{
  mpu6050_data_t mpu6050_data_new;
  mpu6050.readAccel(&mpu6050_data_new.accel_x,
                    &mpu6050_data_new.accel_y,
                    &mpu6050_data_new.accel_z);
  mpu6050.readGyro(&mpu6050_data_new.gyro_x,
                   &mpu6050_data_new.gyro_y,
                   &mpu6050_data_new.gyro_z);

#define FLT(a,x,xkm1) (xkm1*a + x*(1-a))
  mpu6050_data.accel_x = FLT(0.95f, mpu6050_data_new.accel_x, mpu6050_data.accel_x);
  mpu6050_data.accel_y = FLT(0.95f, mpu6050_data_new.accel_y, mpu6050_data.accel_y);
  mpu6050_data.accel_z = FLT(0.95f, mpu6050_data_new.accel_z, mpu6050_data.accel_z);
  mpu6050_data.gyro_x = FLT(0.95f, mpu6050_data_new.gyro_x, mpu6050_data.gyro_x);
  mpu6050_data.gyro_y = FLT(0.95f, mpu6050_data_new.gyro_y, mpu6050_data.gyro_y);
  mpu6050_data.gyro_z = FLT(0.95f, mpu6050_data_new.gyro_z, mpu6050_data.gyro_z);

}


// ----------------- MAINTENANCE CALLBACKS ----------------- 
void onRxSSID(void* rxSSID)
{
  const char* ssid = (const char*) rxSSID;
  memcpy(wifi_ssid, ssid, min(strlen(ssid), sizeof(wifi_ssid)));

  tryWifiConnection((const char*) wifi_ssid, (const char*) wifi_pwd);
}


void onRxPWD(void* rxPWD)
{
  const char* pwd = (const char*) rxPWD;
  memcpy(wifi_pwd, pwd, min(strlen(pwd), sizeof(wifi_pwd)));

  tryWifiConnection((const char*) wifi_ssid, (const char*) wifi_pwd);
}


void onRxGetPWD(void* rxGetPWD)
{
  Serial.print("[OK] "); Serial.println(wifi_pwd);
}

void onRxGetSSID(void* rxGetSSID)
{
  Serial.print("[OK] "); Serial.println(wifi_ssid);
}

void onRxGetIP(void* rxGetIP)
{
  Serial.print("[OK] "); Serial.println(udpServer->localIp());
}

void onRxImuCalib(void* rxImuCalib)
{
  uint8_t loops = atoi((const char*) rxImuCalib) & 0xFF;
  Serial.print("[INFO] <Calibrating IMU> loops("); Serial.print(loops); Serial.println(")");
  mpu6050.gyroByas(loops);
  Serial.println("[OK] Calibration done");
}
// ----------------- MAINTENANCE CALLBACKS ----------------- 


// ----------------- UDP SERVER CALLBACKS ------------------ 
void onGetGyro(void* data)
{
  Vec3D gyro;
  gyro.fields.x = mpu6050_data.gyro_x;
  gyro.fields.y = mpu6050_data.gyro_y;
  gyro.fields.z = mpu6050_data.gyro_z;

  udpServer->answerTo(GET_GYRO_ID, gyro.bytes, sizeof(gyro.bytes));
}


void onGetAccel(void* data)
{
  Vec3D accel;
  accel.fields.x = mpu6050_data.accel_x;
  accel.fields.y = mpu6050_data.accel_y;
  accel.fields.z = mpu6050_data.accel_z;

  udpServer->answerTo(GET_ACCEL_ID, accel.bytes, sizeof(accel.bytes));
}


void onGetAttitude(void* data)
{
  Vec3D attitude;
  attitude.fields.x = attitudeFilter.getRollDeg();
  attitude.fields.y = attitudeFilter.getPitchDeg();
  attitude.fields.z = attitudeFilter.getYawDeg();

  udpServer->answerTo(GET_ATTITUDE_ID, attitude.bytes, sizeof(attitude.bytes));
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

// -------- MAINTENANCE INIT --------
  maint = new Maintenance(115200);

  maint->addCommandCallback(SET_SSID_CMD_ID, onRxSSID);
  maint->addCommandCallback(SET_PWD_CMD_ID, onRxPWD);
  maint->addCommandCallback(GET_PWD_CMD_ID, onRxGetPWD);
  maint->addCommandCallback(GET_SSID_CMD_ID, onRxGetSSID);
  maint->addCommandCallback(GET_IP_CMD_ID, onRxGetIP);
  maint->addCommandCallback(IMU_CALIB_CMD_ID, onRxImuCalib);
// -------- MAINTENANCE INIT --------

// -------- MPU6050 INIT --------
  mpu6050_data.gyro_x = 0x00;
  mpu6050_data.gyro_y = 0x00;
  mpu6050_data.gyro_z = 0x00;
  mpu6050_data.accel_x = 0x00;
  mpu6050_data.accel_y = 0x00;
  mpu6050_data.accel_z = 0x00;
  mpu6050_data.update_t = -1;
  
  Wire.begin(SDA_PIN, SCL_PIN);
  if(!mpu6050.init())
  {
    while(1){
    Serial.println("diocane");
    delay(100);
    }
  };
  mpu6050.gyroByas();
// -------- MPU6050 INIT --------

// -------- UDP SERVER INIT --------
  udpServer = new UDPServer();
  
  udpServer->addMessageCallback(GET_ACCEL_ID, onGetAccel);
  udpServer->addMessageCallback(GET_GYRO_ID,  onGetGyro);
  udpServer->addMessageCallback(GET_ATTITUDE_ID, onGetAttitude);
  udpServer->addMessageCallback(CTRL_ID, onControlCommand);
  
  memset(wifi_ssid, 0x00, sizeof(wifi_ssid));
  memset(wifi_pwd, 0x00, sizeof(wifi_pwd));
// -------- UDP SERVER INIT --------

// -------- FLASH SETTINGS INIT --------
  preferences.begin("storage", false);
  size_t ssid_len = preferences.getString("ssid", wifi_ssid, sizeof(wifi_ssid));
  size_t pwd_len = preferences.getString("pwd", wifi_pwd, sizeof(wifi_pwd));

  if (ssid_len == 0 || pwd_len == 0)
  {
    Serial.println("[INFO] Not read any valid config");
  }
  else
  {
    tryWifiConnection((const char*) wifi_ssid, (const char*) wifi_pwd);
  }
  preferences.end();
// -------- FLASH SETTINGS INIT --------
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
  if ((mpu6050_data.update_t < 0) || ((cur_t_micros - mpu6050_data.update_t) >= (IMU_UPDATE_MILLIS * 1000)))
  {
    mpu6050_data.update_t = cur_t_micros;

    imuUpdate();

    attitudeFilter.update(mpu6050_data.accel_x,
                          mpu6050_data.accel_y,
                          mpu6050_data.accel_z,
                          radians(mpu6050_data.gyro_x),
                          radians(mpu6050_data.gyro_y),
                          radians(mpu6050_data.gyro_z),
                          IMU_UPDATE_MILLIS / 1000.0f);
  }
// ------- IMU UPDATE -------

// ------- MAINTENANCE UPDATE -------
  maint->update();
// ------- MAINTENANCE UPDATE -------
}
