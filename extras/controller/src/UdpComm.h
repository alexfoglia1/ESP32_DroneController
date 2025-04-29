#ifndef UDPCOMM_H
#define UDPCOMM_H

#include <stdint.h>
#include <qudpsocket.h>
#include <qtimer.h>
#include <qthread.h>
#include <qmutex.h>


#pragma pack(push, 1)

#define CTRL_ID         0x00

#define GET_ACCEL_ID    0x01
#define GET_GYRO_ID     0x02
#define GET_ATTITUDE_ID 0x03
#define GET_RPID_ID     0x04
#define GET_PPID_ID     0x05
#define GET_STATUS_ID   0x06

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
    } fields;
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
    } fields;
    float fvec[4];
    uint8_t bytes[16];
} PidVec;

typedef union
{
    struct
    {
        uint8_t MOTOR_1;
        uint8_t MOTOR_2;
        uint8_t MOTOR_3;
        uint8_t MOTOR_4;
        uint8_t throttle_sp;
        float   roll_sp;
        float   pitch_sp;
    } fields;
    uint8_t bytes[13];
} status_msg_t;

typedef struct
{
    uint8_t throttle;
    Vec3D   set_point;
} ctrl_msg_t;


class UdpComm : public QObject
{
    Q_OBJECT

public:

    enum class GetFlag : uint32_t
    {
        ACCEL = 0x01,
        GYRO  = 0x02,
        ATTITUDE = 0x04,
        ROLL_PID = 0x08,
        PITCH_PID = 0x10,
        STATUS = 0x20
    };

    UdpComm();
    bool listen(short port);
    void unlisten();
    void setGetEnabled(GetFlag flag, bool enabled);
    void start(const QString& addr, short port);
    void stop();
    void updateCommand(uint8_t throttle, float roll, float pitch);

signals:
    void receivedRawAccel(float ax, float ay, float az);
    void receivedRawGyro(float gx, float gy, float gz);
    void receivedAttitude(float roll, float pitch, float yaw);
    void receivedRollPid(float P, float I, float D, float U);
    void receivedPitchPid(float P, float I, float D, float U);
    void receivedStatus(uint8_t M1, uint8_t M2, uint8_t M3, uint8_t M4, uint8_t throttle_sp, float roll_sp, float pitch_sp);

    void uplink();
    void downlink();

private:
    
    QUdpSocket* _udpSocket;
    QThread* _txThread;
    QTimer* _txTimer;
    QMutex _dataMutex;
    uint32_t _getFlag;
    QHostAddress _txAddr;
    qint64 _txTimestamp;
    qint64 _rxTimestamp;
    short _txPort;

    uint8_t _throttle;
    float _cmdRoll;
    float _cmdPitch;

    void txControlMessage(uint8_t throttle, float cmdRoll, float cmdPitch);

private slots:
    void onTxTimerTimeout();
    void onSocketReadyRead();


};


#pragma pack(pop)
#endif // UDPCOMM_H
