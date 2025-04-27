#include "UdpComm.h"

#include <qdatetime.h>

UdpComm::UdpComm()
{
	_getFlag = 0;

	_udpSocket = nullptr;
	_txThread = nullptr;

	_txTimer = nullptr;

	_txTimestamp = 0;
	_rxTimestamp = 0;
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


void UdpComm::onTxTimerTimeout()
{
	qint64 cur_t = QDateTime::currentMSecsSinceEpoch();

	uint32_t getFlag = 0;
	_dataMutex.lock();
	getFlag = _getFlag;
	_dataMutex.unlock();

	QByteArray qba;

	bool tx = false;
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ACCEL))
	{
		tx = true;

		qba.push_back(GET_ACCEL_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::GYRO))
	{
		tx = true;

		qba.push_back(GET_GYRO_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::ATTITUDE))
	{
		tx = true;

		qba.push_back(GET_ATTITUDE_ID);
		_udpSocket->writeDatagram(qba, _txAddr, _txPort);
	}
	if (static_cast<uint32_t>(getFlag) & static_cast<uint32_t>(UdpComm::GetFlag::PID))
	{
		tx = true;

		qba.push_back(GET_PID_ID);
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
