#include "SerialComm.h"

SerialComm::SerialComm()
{
	_serialPort = nullptr;
}


bool SerialComm::open(const QString& serialPortName, QSerialPort::BaudRate baudRate)
{
    if (_serialPort)
    {
        close();
    }

    _serialPort = new QSerialPort();
    _serialPort->setPortName(serialPortName);
    _serialPort->setBaudRate(baudRate);
    _serialPort->setParity(QSerialPort::NoParity);
    _serialPort->setDataBits(QSerialPort::Data8);
    _serialPort->setStopBits(QSerialPort::OneStop);
    _serialPort->setFlowControl(QSerialPort::NoFlowControl);
   
    connect(_serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));

    bool ret = _serialPort->open(QSerialPort::OpenModeFlag::ReadWrite);

    //_serialPort->setDataTerminalReady(true);
    //_serialPort->setRequestToSend(true);

    return ret;
}


bool SerialComm::close()
{
    if (_serialPort)
    {
        disconnect(_serialPort, SIGNAL(readyRead()), this, SLOT(onReadyRead()));

        _serialPort->close();
        _serialPort->deleteLater();
        _serialPort = nullptr;

        return true;
    }
    else
    {
        return false;
    }
}


void SerialComm::sendMessage(const QString& message)
{
    if (_serialPort)
    {
        QByteArray qba;
        for (auto byte : message.toLocal8Bit())
        {
            qba.push_back(byte);
        }
        qba.push_back('\0');
        _serialPort->write(qba);
        _serialPort->flush();
    }
}


void SerialComm::onReadyRead()
{
    QByteArray qba = _serialPort->readAll();
    emit messageReceived(QString(qba.toStdString().c_str()));
}