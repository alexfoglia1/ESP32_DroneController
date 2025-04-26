#include "UdpComm.h"

UdpComm::UdpComm()
{
	_getFlag = 0;

	_udpSocket = new QUdpSocket(this);

	connect(_udpSocket, SIGNAL(readyRead()), this, SLOT(onSocketReadyRead()));

	_txThread = new QThread();

	_txTimer = new QTimer();
	_txTimer->setSingleShot(false);
	_txTimer->setTimerType(Qt::PreciseTimer);
	_txTimer->setInterval(100);

	connect(_txTimer, SIGNAL(timeout()), this, SLOT(onTxTimerTimeout()));
	connect(_txThread, SIGNAL(started()), _txTimer, SLOT(start()));
	
	_txTimer->moveToThread(_txThread);
}


bool UdpComm::listen(short port)
{
	return _udpSocket->bind(QHostAddress::AnyIPv4, port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
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
	_txPort = port;
	_txAddr = QHostAddress(address);

	_txThread->start();
}

void UdpComm::stop()
{
	// TODO
}


void UdpComm::onTxTimerTimeout()
{
	uint32_t getFlag = 0;
	_dataMutex.lock();
	getFlag = _getFlag;
	_dataMutex.unlock();

	QByteArray qba;
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ACCEL))
	{
		qba.push_back(GET_ACCEL_ID);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::GYRO))
	{
		qba.push_back(GET_GYRO_ID);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ATTITUDE))
	{
		qba.push_back(GET_ATTITUDE_ID);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::PID))
	{
		qba.push_back(GET_PID_ID);
	}

	if (qba.size() == 0) return;

	_udpSocket->writeDatagram(qba, _txAddr, _txPort);
}


void UdpComm::onSocketReadyRead()
{
	while (_udpSocket->hasPendingDatagrams())
	{
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
		case GET_PID_ID: emit receivedPid(*reinterpret_cast<float*>(&msg_in->payload[0]),
											   *reinterpret_cast<float*>(&msg_in->payload[4]),
											   *reinterpret_cast<float*>(&msg_in->payload[8]),
											   *reinterpret_cast<float*>(&msg_in->payload[12]));
		break;
		default:
		break;
		}
	}
}
