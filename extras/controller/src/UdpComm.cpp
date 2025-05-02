#include "UdpComm.h"

#include <qdatetime.h>

UdpComm::UdpComm()
{
	_getFlag = (uint8_t)(UdpComm::GetFlag::STATUS);

	_udpSocket = nullptr;
	_txThread = nullptr;

	_txTimer = nullptr;

	_throttle = 0;
	_cmdRoll = 0.0f;
	_cmdPitch = 0.0f;

	_txTimestamp = 0;
	_rxTimestamp = 0;

	_connected = false;
}


bool UdpComm::listen(short port)
{
	if (_udpSocket)
	{
		unlisten();
	}

	_udpSocket = new QUdpSocket(this);

	connect(_udpSocket, SIGNAL(readyRead()), this, SLOT(onSocketReadyRead()));
	return _udpSocket->bind(QHostAddress::AnyIPv4, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
}

void UdpComm::unlisten()
{
	if (_udpSocket)
	{
		disconnect(_udpSocket, SIGNAL(readyRead()), this, SLOT(onSocketReadyRead()));
		_udpSocket->close();
		_udpSocket->deleteLater();
		_udpSocket = nullptr;

		_connected = false;
	}
}


void UdpComm::setGetEnabled(UdpComm::GetFlag flag, bool enabled)
{
	_dataMutex.lock();

	if (enabled)
	{
		_getFlag |= static_cast<uint32_t>(flag);
	}
	else
	{
		_getFlag &= ~(static_cast<uint32_t>(flag));
	}

	_dataMutex.unlock();
}


void UdpComm::start(const QString& address, short port)
{
	listen(port);

	_txPort = port;
	_txAddr = QHostAddress(address);

	if (_txThread != nullptr || _txTimer != nullptr)
	{
		stop();
	}

	_txThread = new QThread();

	_txTimer = new QTimer();
	_txTimer->setSingleShot(false);
	_txTimer->setTimerType(Qt::PreciseTimer);
	_txTimer->setInterval(100);

	_txTimer->moveToThread(_txThread);

	connect(_txTimer, SIGNAL(timeout()), this, SLOT(onTxTimerTimeout()));
	connect(_txThread, SIGNAL(started()), _txTimer, SLOT(start()));

	_connected = true;

	_txThread->start();
}

void UdpComm::stop()
{
	if (_txThread && _txTimer)
	{
		unlisten();

		disconnect(_txTimer, SIGNAL(timeout()), this, SLOT(onTxTimerTimeout()));
		disconnect(_txThread, SIGNAL(started()), _txTimer, SLOT(start()));

		_txThread->terminate();

		_txThread->deleteLater();
		_txThread = nullptr;

		_txTimer->deleteLater();
		_txTimer = nullptr;

		emit downlink();
	}
}


void UdpComm::updateCommand(uint8_t throttle, float cmdRoll, float cmdPitch)
{
	_dataMutex.lock();

	_throttle = throttle;
	_cmdRoll = cmdRoll;
	_cmdPitch = cmdPitch;
	
	_dataMutex.unlock();
}


void UdpComm::sendArmedCommand(bool isArmed)
{
	if (!_connected) return;

	_dataMutex.lock();

	general_msg_t armed_cmd;
	armed_cmd.msg_id = ARM_MOTORS_ID;
	armed_cmd.payload[0] = isArmed ? 1 : 0;

	_udpSocket->writeDatagram((const char*)&armed_cmd, 2, _txAddr, _txPort);

	_dataMutex.unlock();
}


void UdpComm::testMotors(UdpComm::MotorFlag motors_flags, uint8_t pwm)
{
	if (!_connected) return;

	_dataMutex.lock();

	general_msg_t test_motors_gen_msg;
	test_motors_gen_msg.msg_id = TEST_MOTORS_ID;
	
	test_motors_msg_t* test_motors_msg = reinterpret_cast<test_motors_msg_t*>(&test_motors_gen_msg.payload);
	test_motors_msg->fields.motors_flag = static_cast<uint8_t>(motors_flags);
	test_motors_msg->fields.throttle = pwm;

	_udpSocket->writeDatagram((const char*)&test_motors_gen_msg, 1 + sizeof(test_motors_msg_t), _txAddr, _txPort);

	_dataMutex.unlock();
}


void UdpComm::setRollPid(float kp, float ki, float kd)
{
	if (!_connected) return;

	_dataMutex.lock();

	general_msg_t roll_pid_gen_msg;
	roll_pid_gen_msg.msg_id = SET_ROLL_PID_ID;

	PidParamsVec* pid_params_vec = reinterpret_cast<PidParamsVec*>(&roll_pid_gen_msg.payload);
	
	pid_params_vec->fields.Kp = kp;
	pid_params_vec->fields.Ki = ki;
	pid_params_vec->fields.Kd = kd;

	_udpSocket->writeDatagram((const char*)&roll_pid_gen_msg, 1 + sizeof(PidParamsVec), _txAddr, _txPort);

	_dataMutex.unlock();
}


void UdpComm::setPitchPid(float kp, float ki, float kd)
{
	if (!_connected) return;

	_dataMutex.lock();

	general_msg_t pitch_pid_gen_msg;
	pitch_pid_gen_msg.msg_id = SET_PITCH_PID_ID;

	PidParamsVec* pid_params_vec = reinterpret_cast<PidParamsVec*>(&pitch_pid_gen_msg.payload);

	pid_params_vec->fields.Kp = kp;
	pid_params_vec->fields.Ki = ki;
	pid_params_vec->fields.Kd = kd;

	_udpSocket->writeDatagram((const char*)&pitch_pid_gen_msg, 1 + sizeof(PidParamsVec), _txAddr, _txPort);

	_dataMutex.unlock();
}


void UdpComm::txControlMessage(uint8_t throttle, float cmdRoll, float cmdPitch)
{
	general_msg_t cmd_gen_msg;
	cmd_gen_msg.msg_id = CTRL_ID;

	ctrl_msg_t* cmd_ctrl_msg = reinterpret_cast<ctrl_msg_t*>(&cmd_gen_msg.payload);
	cmd_ctrl_msg->throttle = throttle;
	cmd_ctrl_msg->set_point.fields.x = cmdRoll;
	cmd_ctrl_msg->set_point.fields.y = cmdPitch;
	cmd_ctrl_msg->set_point.fields.z = 0.0f;

	_udpSocket->writeDatagram((const char*)&cmd_gen_msg, 1 + sizeof(ctrl_msg_t), _txAddr, _txPort);
}


void UdpComm::onTxTimerTimeout()
{
	qint64 cur_t = QDateTime::currentMSecsSinceEpoch();

	uint8_t throttle;
	float cmdRoll;
	float cmdPitch;

	uint32_t getFlag = 0;
	_dataMutex.lock();
	getFlag = _getFlag;
	throttle = _throttle;
	cmdRoll = _cmdRoll;
	cmdPitch = _cmdPitch;
	_dataMutex.unlock();

	txControlMessage(throttle, cmdRoll, cmdPitch);

	bool tx = false;
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ACCEL))
	{
		tx = true;

		QByteArray qba;
		qba.push_back(GET_ACCEL_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::GYRO))
	{
		tx = true;

		QByteArray qba;
		qba.push_back(GET_GYRO_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ATTITUDE))
	{
		tx = true;

		QByteArray qba;
		qba.push_back(GET_ATTITUDE_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ROLL_PID))
	{
		tx = true;

		QByteArray qba;
		qba.push_back(GET_RPID_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::PITCH_PID))
	{
		tx = true;

		QByteArray qba;
		qba.push_back(GET_PPID_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::STATUS))
	{
		tx = true;

		QByteArray qba;
		qba.push_back(GET_STATUS_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}

	if (tx)
	{
		qint64 deltaT = _txTimestamp - _rxTimestamp;

		if (deltaT > 1000)
		{
			emit downlink();
		}

		_txTimestamp = cur_t;
	}
	
}


void UdpComm::onSocketReadyRead()
{
	qint64 cur_t = QDateTime::currentMSecsSinceEpoch();

	while (_udpSocket->hasPendingDatagrams())
	{
		_rxTimestamp = cur_t;

		emit uplink();

		char* dataIn = new char[sizeof(general_msg_t)];
		_udpSocket->readDatagram(dataIn, sizeof(general_msg_t));

		general_msg_t* msg_in = reinterpret_cast<general_msg_t*>(dataIn);


		switch (msg_in->msg_id)
		{
		case GET_ACCEL_ID: emit receivedRawAccel(*reinterpret_cast<float*>(&msg_in->payload[0]),
												 *reinterpret_cast<float*>(&msg_in->payload[4]),
												 *reinterpret_cast<float*>(&msg_in->payload[8]));
		break;
		case GET_GYRO_ID:  emit receivedRawGyro(*reinterpret_cast<float*>(&msg_in->payload[0]),
												*reinterpret_cast<float*>(&msg_in->payload[4]),
												*reinterpret_cast<float*>(&msg_in->payload[8]));
		break;
		case GET_ATTITUDE_ID: emit receivedAttitude(*reinterpret_cast<float*>(&msg_in->payload[0]),
													*reinterpret_cast<float*>(&msg_in->payload[4]),
													*reinterpret_cast<float*>(&msg_in->payload[8]));
		break;
		case GET_RPID_ID: emit receivedRollPid(*reinterpret_cast<float*>(&msg_in->payload[0]),
											   *reinterpret_cast<float*>(&msg_in->payload[4]),
											   *reinterpret_cast<float*>(&msg_in->payload[8]),
											   *reinterpret_cast<float*>(&msg_in->payload[12]));
		break;
		case GET_PPID_ID: emit receivedPitchPid(*reinterpret_cast<float*>(&msg_in->payload[0]),
			*reinterpret_cast<float*>(&msg_in->payload[4]),
			*reinterpret_cast<float*>(&msg_in->payload[8]),
			*reinterpret_cast<float*>(&msg_in->payload[12]));
		break;
		case GET_STATUS_ID:
		{
			status_msg_t* status_msg_in = reinterpret_cast<status_msg_t*>(&msg_in->payload[0]);
			emit receivedStatus(status_msg_in->fields.MOTOR_1,
								status_msg_in->fields.MOTOR_2,
								status_msg_in->fields.MOTOR_3,
								status_msg_in->fields.MOTOR_4,
								status_msg_in->fields.throttle_sp,
								status_msg_in->fields.roll_sp,
								status_msg_in->fields.pitch_sp);
		}
		break;
		default:
		break;
		}
	}
}
