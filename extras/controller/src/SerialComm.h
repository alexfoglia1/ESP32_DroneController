#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <qserialport.h>

class SerialComm : public QObject
{
	Q_OBJECT
public:
	SerialComm();

	bool open(const QString& serialPortName, const QSerialPort::BaudRate baudRate);
	bool close();

public slots:
	void sendMessage(const QString& message);
	
signals:
	void messageReceived(const QString& message);

private slots:
	void onReadyRead();

private:
	QSerialPort* _serialPort;
};

#endif
