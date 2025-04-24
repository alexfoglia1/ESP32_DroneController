
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define MOTOR_1 5

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const char *ssid = "Vodafone-A38373416";
const char *password = "nhqhsf6CFtTYm4FT";


typedef struct
{
  uint32_t magic;
  uint8_t  motor;
  uint8_t  pwm;
} __attribute__((packed)) cmd_t;

void setup() {
//    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(8, 9);
    //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    //    Fastwire::setup(400, true);
//#endif
        accelgyro.initialize();
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }
  if (udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    #if 1
    udp.onPacket([](AsyncUDPPacket packet) {
      uint8_t* pData = packet.data();
      uint32_t len = packet.length();
      cmd_t* pCmd = (cmd_t*)(pData);
      if (len >= sizeof(cmd_t) && pCmd->magic == 0xb0bafe77)
      {
        int motors[4] = {MOTOR_1, MOTOR_1, MOTOR_1, MOTOR_1};
        analogWrite(motors[pCmd->motor & 3], pCmd->pwm);
        packet.printf("OK!\n");
      }
      else
      {
        packet.printf("NOK!\n");
      }

      Serial.print("UDP Packet Type: ");
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast");
      Serial.print(", From: ");
      Serial.print(packet.remoteIP());
      Serial.print(":");
      Serial.print(packet.remotePort());
      Serial.print(", To: ");
      Serial.print(packet.localIP());
      Serial.print(":");
      Serial.print(packet.localPort());
      Serial.print(", Length: ");
      Serial.print(packet.length());
      Serial.print(", Data: ");
      Serial.write(packet.data(), packet.length());
      Serial.println();
      //reply to the client
      packet.printf("Got %u bytes of data", packet.length());

    });
#endif
  }
}


typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  int16_t gx;
  int16_t gy;
  int16_t gz;
} __attribute__((packed)) accel_msg_t;


void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accel_msg_t msg_out;
  msg_out.ax = ax;
  msg_out.ay = ay;
  msg_out.az = az;
  msg_out.gx = gx;
  msg_out.gy = gy;
  msg_out.gz = gz;

  udp.broadcast((uint8_t*) &msg_out, sizeof(accel_msg_t));
  delay(100);
}
