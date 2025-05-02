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

#define IMU_UPDATE_MILLIS 5

/** Remapped to have M1 M2
                       X
                     M4 M3
**/
#define MOTOR_1 21
#define MOTOR_2 5
#define MOTOR_3 6
#define MOTOR_4 20

#define IMU_PWR 10

const float KP_ROLL = 1.0f;
const float KI_ROLL = 0.0f;
const float KD_ROLL = 0.0f;

const float KP_PITCH = 1.0f;
const float KI_PITCH = 0.0f;
const float KD_PITCH = 0.0f;

PidVec rollPid;
PidVec pitchPid;
float rollErrBefore;
float pitchErrBefore;

const int MOTOR_PINS[4] = {MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4};

int64_t loop_time;
int64_t max_loop_time;

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

bool test_motors;
uint8_t throttle_setpoint;
float roll_setpoint;
float pitch_setpoint;
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
  mpu6050_data.accel_x = FLT(0.60f, mpu6050_data_new.accel_x, mpu6050_data.accel_x);
  mpu6050_data.accel_y = FLT(0.60f, mpu6050_data_new.accel_y, mpu6050_data.accel_y);
  mpu6050_data.accel_z = FLT(0.60f, mpu6050_data_new.accel_z, mpu6050_data.accel_z);
  mpu6050_data.gyro_x = FLT(0.98f, mpu6050_data_new.gyro_x, mpu6050_data.gyro_x);
  mpu6050_data.gyro_y = FLT(0.98f, mpu6050_data_new.gyro_y, mpu6050_data.gyro_y);
  mpu6050_data.gyro_z = FLT(0.98f, mpu6050_data_new.gyro_z, mpu6050_data.gyro_z);

}


// ----------------- MAINTENANCE CALLBACKS ----------------- 
void onRxSSID(void* rxSSID)
{
  const char* ssid = (const char*) rxSSID;

  memset(wifi_ssid, 0x00, sizeof(wifi_ssid));
  memcpy(wifi_ssid, ssid, min(strlen(ssid), sizeof(wifi_ssid)));

  tryWifiConnection((const char*) wifi_ssid, (const char*) wifi_pwd);
}


void onRxPWD(void* rxPWD)
{
  const char* pwd = (const char*) rxPWD;

  memset(wifi_pwd, 0x00, sizeof(wifi_pwd));
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
  int loops = atoi((const char*) rxImuCalib);
  Serial.print("[INFO] <Calibrating IMU> loops("); Serial.print(loops); Serial.println(")");
  mpu6050.gyroByas(loops);
  Serial.println("[OK] Calibration done");
}


void onRxMotorCmdId(void* rxMotorCmd)
{
  char* rxMotorCmdStr = (char*) rxMotorCmd;
  char rxMotor = rxMotorCmdStr[0];
  char* rxPwm = (char*)(&rxMotorCmdStr[1]);
  uint8_t motor = atoi((const char*)&rxMotor) % 5;
  uint8_t pwm = atoi(rxPwm) & 0xFF;
  Serial.print("[INFO] <Set motor> motor("); Serial.print(motor); Serial.print(") pwm("); Serial.print(pwm); Serial.println(")");

  switch (motor)
  {
    case 1: test_motors = true; ledcWrite(MOTOR_1, pwm); break;
    case 2: test_motors = true; ledcWrite(MOTOR_2, pwm); break;
    case 3: test_motors = true; ledcWrite(MOTOR_3, pwm); break;
    case 4: test_motors = true; ledcWrite(MOTOR_4, pwm); break;
    default: Serial.println("[INFO} Test motors mode disabled"); test_motors = false; break;
  }
}


void onRxGetLoopTime(void* rxGetLoopTime)
{
  Serial.print("[OK] <LOOP TIME> "); Serial.print(loop_time); Serial.println(" micros");
}


void onRxGetMaxLoopTime(void* rxGetMaxLoopTime)
{
  Serial.print("[OK] <MAX LOOP TIME> "); Serial.print(max_loop_time); Serial.println(" micros");
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


void onGetPitchPid(void* data)
{
  udpServer->answerTo(GET_PPID_ID, pitchPid.bytes, sizeof(pitchPid.bytes));
}


void onGetRollPid(void* data)
{
  udpServer->answerTo(GET_RPID_ID, rollPid.bytes, sizeof(rollPid.bytes));
}


void onControlCommand(void* data)
{
  general_msg_t* msgIn = (general_msg_t*) data;
  ctrl_msg_t* ctrlCmdIn = (ctrl_msg_t*) &msgIn->payload[0];

  throttle_setpoint = ctrlCmdIn->throttle;
  // TODO: saturate at min angle max angle
  roll_setpoint = ctrlCmdIn->set_point.fields.x;
  pitch_setpoint = ctrlCmdIn->set_point.fields.y;
}


void onGetStatusMessage(void* data)
{
  status_msg_t msgOut;
  msgOut.fields.M1 = throttle_status.motor_1_throttle;
  msgOut.fields.M2 = throttle_status.motor_2_throttle;
  msgOut.fields.M3 = throttle_status.motor_3_throttle;
  msgOut.fields.M4 = throttle_status.motor_4_throttle;
  msgOut.fields.throttle_sp = throttle_setpoint;
  msgOut.fields.roll_sp = roll_setpoint;
  msgOut.fields.pitch_sp = pitch_setpoint;

  udpServer->answerTo(GET_STATUS_ID, msgOut.bytes, sizeof(msgOut.bytes));
}
// ----------------- UDP SERVER CALLBACKS ------------------ 


// ----------------- PID CONTROLLER -----------------
void PID(PidVec& pid, float& err_before, float y, float ysp, float KP, float KI, float KD, float dt)
{
  float err = ysp - y;
  float dErr = (err - err_before) / dt;
  float iErr = (err_before + err) * dt;
  err_before = err;

  pid.fields.P = err * KP;
  //TODO : Anti-windup
  pid.fields.I = iErr * KI;
  pid.fields.D = dErr * KD;

  pid.fields.U = (pid.fields.P + pid.fields.I + pid.fields.D);
}


void pid_controller(uint8_t throttle_sp, float roll_sp, float roll, float pitch_sp, float pitch)
{
  PID(rollPid, rollErrBefore, roll, roll_sp, KP_ROLL, KI_ROLL, KD_ROLL, IMU_UPDATE_MILLIS / 1000.0f);
  PID(pitchPid, pitchErrBefore, pitch, pitch_sp, KP_PITCH, KI_PITCH, KD_PITCH, IMU_UPDATE_MILLIS / 1000.0f);

  float m1f = throttle_sp - rollPid.fields.U + pitchPid.fields.U;
  float m2f = throttle_sp + rollPid.fields.U + pitchPid.fields.U;
  float m3f = throttle_sp + rollPid.fields.U - pitchPid.fields.U;
  float m4f = throttle_sp - rollPid.fields.U - pitchPid.fields.U;

  throttle_command.motor_1_throttle = m1f > 255 ? 255 : m1f < 0 ? 0 : (uint8_t)(m1f);
  throttle_command.motor_2_throttle = m2f > 255 ? 255 : m2f < 0 ? 0 : (uint8_t)(m2f);
  throttle_command.motor_3_throttle = m3f > 255 ? 255 : m3f < 0 ? 0 : (uint8_t)(m3f);
  throttle_command.motor_4_throttle = m4f > 255 ? 255 : m4f < 0 ? 0 : (uint8_t)(m4f);
}
// ----------------- PID CONTROLLER -----------------


void setup()
{
  pinMode(IMU_PWR, OUTPUT);
  digitalWrite(IMU_PWR, LOW);

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
  roll_setpoint = 0;
  pitch_setpoint = 0;
// -------- MOTORS INIT ----------

  delay(1000);

// -------- MAINTENANCE INIT --------
  maint = new Maintenance(115200);

  maint->addCommandCallback(SET_SSID_CMD_ID, onRxSSID);
  maint->addCommandCallback(SET_PWD_CMD_ID, onRxPWD);
  maint->addCommandCallback(GET_PWD_CMD_ID, onRxGetPWD);
  maint->addCommandCallback(GET_SSID_CMD_ID, onRxGetSSID);
  maint->addCommandCallback(GET_IP_CMD_ID, onRxGetIP);
  maint->addCommandCallback(GET_LOOP_T_CMD_ID, onRxGetLoopTime);
  maint->addCommandCallback(GET_MAX_LPT_CMD_ID, onRxGetMaxLoopTime);
  maint->addCommandCallback(IMU_CALIB_CMD_ID, onRxImuCalib);
  maint->addCommandCallback(MOTOR_CMD_ID, onRxMotorCmdId);
// -------- MAINTENANCE INIT --------

// -------- MPU6050 INIT --------
  mpu6050_data.gyro_x = 0x00;
  mpu6050_data.gyro_y = 0x00;
  mpu6050_data.gyro_z = 0x00;
  mpu6050_data.accel_x = 0x00;
  mpu6050_data.accel_y = 0x00;
  mpu6050_data.accel_z = 0x00;
  mpu6050_data.update_t = -1;
  
  Wire.begin(SDA_PIN, SCL_PIN, 400000);
  Wire.setClock(400000);

  int maxRetry = 10;
  int nRetry = 0;
  bool mpuInit = false;
  while(!mpuInit && nRetry < maxRetry)
  {
    mpuInit = mpu6050.init();

    if (!mpuInit)
    {
      Serial.println("[NOK] IMU INIT, Retry");
      delay(100);
    }
    else
    {
      Serial.println("[OK] IMU INIT");
    }

    nRetry += 1;
  };

  mpu6050.gyroByas();

// -------- MPU6050 INIT --------

// -------- UDP SERVER INIT --------
  udpServer = new UDPServer();
  
  udpServer->addMessageCallback(GET_ACCEL_ID, onGetAccel);
  udpServer->addMessageCallback(GET_GYRO_ID,  onGetGyro);
  udpServer->addMessageCallback(GET_ATTITUDE_ID, onGetAttitude);
  udpServer->addMessageCallback(GET_PPID_ID, onGetPitchPid);
  udpServer->addMessageCallback(GET_RPID_ID, onGetRollPid);
  udpServer->addMessageCallback(CTRL_ID, onControlCommand);
  udpServer->addMessageCallback(GET_STATUS_ID, onGetStatusMessage);
  
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

  loop_time = 0;
  max_loop_time = 0;
  test_motors = false;

  rollPid.fields.P = 0;
  rollPid.fields.I = 0;
  rollPid.fields.D = 0;
  rollPid.fields.U = 0;

  pitchPid.fields.P = 0;
  pitchPid.fields.I = 0;
  pitchPid.fields.D = 0;
  pitchPid.fields.U = 0;

  rollErrBefore = 0;
  pitchErrBefore = 0;
}


void loop()
{
  int64_t cur_t_micros = micros();
// ------- MOTORS UPDATE -------
  if (!test_motors)
  {
    ledcWrite(MOTOR_1, throttle_command.motor_1_throttle);
    ledcWrite(MOTOR_2, throttle_command.motor_2_throttle);
    ledcWrite(MOTOR_3, throttle_command.motor_3_throttle);
    ledcWrite(MOTOR_4, throttle_command.motor_4_throttle);
  }

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
    //memset(&mpu6050_data, 0x00, sizeof(mpu6050_data));

    attitudeFilter.update(mpu6050_data.accel_x,
                          mpu6050_data.accel_y,
                          mpu6050_data.accel_z,
                          radians(mpu6050_data.gyro_x),
                          radians(mpu6050_data.gyro_y),
                          radians(mpu6050_data.gyro_z),
                          IMU_UPDATE_MILLIS / 1000.0f);

    pid_controller(throttle_setpoint, roll_setpoint, attitudeFilter.getRollDeg(), pitch_setpoint, attitudeFilter.getPitchDeg());
  }
// ------- IMU UPDATE -------

// ------- LOOP TIME UPDATE -------
  loop_time = micros() - cur_t_micros;
  if (loop_time > max_loop_time)
  {
    max_loop_time = loop_time;
  }
// ------- LOOP TIME UPDATE -------

// ------- MAINTENANCE UPDATE -------
  maint->update();
// ------- MAINTENANCE UPDATE -------  
}
