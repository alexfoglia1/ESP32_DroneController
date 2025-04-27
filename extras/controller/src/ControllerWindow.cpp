#include "ControllerWindow.h"

#include <qapplication.h>
#include <qtimer.h>
#include <qserialport.h>
#include <qmessagebox.h>


ControllerWindow::ControllerWindow()
{
	_ui.setupUi(this);
	_progressUi.setupUi(&_autoscanProgressWindow);
	_autoscanProgressWindow.setVisible(false);

	_ui.comboSelBaud->addItem("1200", QSerialPort::Baud1200);
	_ui.comboSelBaud->addItem("2400", QSerialPort::Baud2400);
	_ui.comboSelBaud->addItem("4800", QSerialPort::Baud4800);
	_ui.comboSelBaud->addItem("9600", QSerialPort::Baud9600);
	_ui.comboSelBaud->addItem("19200", QSerialPort::Baud19200);
	_ui.comboSelBaud->addItem("38400", QSerialPort::Baud38400);
	_ui.comboSelBaud->addItem("57600", QSerialPort::Baud57600);
	_ui.comboSelBaud->addItem("115200", QSerialPort::Baud115200);
	_ui.comboSelBaud->setCurrentIndex(7); // Default 115200

	connect(&_js, SIGNAL(joystickCommand(quint8, float, float)), this, SLOT(OnJoystickCommand(quint8, float, float)));

	connect(_ui.btnOpenSerialPort, SIGNAL(clicked()), this, SLOT(OnBtnOpenSerialPort()));
	connect(_ui.btnRescanPorts, SIGNAL(clicked()), this, SLOT(OnBtnRescanPorts()));
	connect(_ui.btnConnect, SIGNAL(clicked()), this, SLOT(OnBtnConnect()));

	connect(_ui.plotTimeSlider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotSliderValueChanged(int)));
	connect(_ui.plotTrack1Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack1ValueChanged(int)));
	connect(_ui.plotTrack2Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack2ValueChanged(int)));
	connect(_ui.plotTrack3Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack3ValueChanged(int)));
	connect(_ui.plotTrack4Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack4ValueChanged(int)));

	connect(_ui.comboSelTrack1, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack1TextChanged(const QString&)));
	connect(_ui.comboSelTrack2, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack2TextChanged(const QString&)));
	connect(_ui.comboSelTrack3, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack3TextChanged(const QString&)));
	connect(_ui.comboSelTrack4, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack4TextChanged(const QString&)));

	connect(_ui.serialTerminalWidget, SIGNAL(commandSent(const QString&)), &_serialComm, SLOT(sendMessage(const QString&)));
	connect(&_serialComm, SIGNAL(messageReceived(const QString&)), _ui.serialTerminalWidget, SLOT(responseReceived(const QString&)));

	// TODO : completare udp comm con signal/slot non lambda
	connect(&_udpComm, &UdpComm::receivedAttitude, this, [this](float roll, float pitch, float yaw) {
		checkPlot("BODY_ROLL", roll);
		checkPlot("BODY_PITCH", pitch);
		checkPlot("BODY_YAW", yaw);

		_ui.pfdRollPitch->UpdateRoll(roll);
		_ui.pfdRollPitch->UpdatePitch(pitch);
		_ui.pfdHeading->UpdateHeading(yaw);
		});

	connect(&_udpComm, &UdpComm::receivedRawAccel, this, [this](float ax, float ay, float az)
		{
			checkPlot("RAW_ACC_X", ax);
			checkPlot("RAW_ACC_Y", ay);
			checkPlot("RAW_ACC_Z", az);
		});

	connect(&_udpComm, &UdpComm::receivedRawGyro, this, [this](float gx, float gy, float gz)
		{
			checkPlot("RAW_GYRO_X", gx);
			checkPlot("RAW_GYRO_Y", gy);
			checkPlot("RAW_GYRO_Z", gz);
		});


	QTimer* autoscanComPortsTimer = new QTimer();
	autoscanComPortsTimer->setSingleShot(true);
	autoscanComPortsTimer->setTimerType(Qt::PreciseTimer);

	connect(autoscanComPortsTimer, &QTimer::timeout, this, [this, autoscanComPortsTimer] { this->autoScanComPorts(); autoscanComPortsTimer->deleteLater(); });
	autoscanComPortsTimer->start(500);

	const double SAMPLE_PERIOD_S = _txDelayMillis * 1e-3;
	const double SAMPLE_FREQ = 1 / SAMPLE_PERIOD_S;

	_ui.plot->UpdateSamplesPerSecond(SAMPLE_FREQ);
	int samplesInNewValue = SAMPLE_FREQ * _ui.plotTimeSlider->value();

	_ui.plot->SetXSpan(samplesInNewValue);

	_ui.plot->ForceRepaint();
	_ui.pfdHeading->ForceRepaint();
	_ui.pfdRollPitch->ForceRepaint();
}


void ControllerWindow::OnJoystickCommand(quint8 throttle, float roll, float pitch)
{
	//TODO
}


void ControllerWindow::OnBtnOpenSerialPort()
{
	if (_ui.btnOpenSerialPort->text().toUpper() == "OPEN")
	{
		int idx = _ui.comboSelBaud->currentIndex();
		enum QSerialPort::BaudRate baud = QSerialPort::Baud38400;
		if (idx != -1)
		{
#ifdef __linux__
			baud = QSerialPort::BaudRate(_ui.comboSelBaud->itemData(idx).toInt());
#else
			baud = enum QSerialPort::BaudRate(_ui.comboSelBaud->itemData(idx).toInt());
#endif
		}

		if (_serialComm.open(_ui.comboSelPort->currentText(), baud))
		{

			_ui.comboSelPort->setEnabled(false);

			_ui.btnOpenSerialPort->setText("Close");
			_ui.btnRescanPorts->setEnabled(false);
		}
		else
		{
			QMessageBox::warning(this, "Error", QString("Cannot open serial port %1").arg(_ui.comboSelPort->currentText()));
		}

	}
	else
	{

		_serialComm.close();
		_ui.comboSelPort->setEnabled(true);
		_ui.btnRescanPorts->setEnabled(true);

		_ui.btnOpenSerialPort->setText("Open");
	}
}


void ControllerWindow::OnBtnRescanPorts()
{
	_ui.comboSelPort->clear();
	autoScanComPorts();
}


void ControllerWindow::OnBtnConnect()
{
	QString addr = _ui.lineIpAddr->text();
	short port = _ui.linePort->text().toShort();
	_ui.linePort->setText(QString::number(port));

	_udpComm.listen(port);
	// TODO sta cosa farla a seconda di cosa voglio, non statica all'avvio
	_udpComm.setGetEnabled(UdpComm::GetFlag::ATTITUDE, true);
	//_udpComm.setGetEnabled(UdpComm::GetFlag::ACCEL, true);
	//_udpComm.setGetEnabled(UdpComm::GetFlag::GYRO, true);

	_udpComm.start(addr, port);
}



void ControllerWindow::autoScanComPorts()
{
	_progressUi.autoscanStatusPrompt->setText("");

	_autoscanProgressWindow.setVisible(true);
	for (int i = 10; i < 100; i++)
	{
#ifdef __linux__
		QString portName = QString("/dev/ttyACM%1").arg(i);
#else
		QString portName = QString("COM%1").arg(i);
#endif
		_progressUi.autoscanStatusPrompt->append(QString("Testing %1").arg(portName));
		qApp->processEvents();

		QSerialPort* serialPort = new QSerialPort(portName);
		if (serialPort->open(QSerialPort::ReadWrite))
		{
			serialPort->close();
			serialPort->deleteLater();

			_ui.comboSelPort->addItem(portName);
			_progressUi.autoscanStatusPrompt->append(QString("OK"));
		}
		else
		{
			_progressUi.autoscanStatusPrompt->append(QString("FAIL"));
		}

		_progressUi.autoscanStatusProgress->setValue(i + 1);

		qApp->processEvents();
	}
	_autoscanProgressWindow.setVisible(false);
}


void ControllerWindow::checkPlot(QString expected, double value)
{
	if (_ui.comboSelTrack1->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(0, value);
	}

	if (_ui.comboSelTrack2->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(1, value);
	}

	if (_ui.comboSelTrack3->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(2, value);
	}

	if (_ui.comboSelTrack4->currentText().toUpper() == expected.toUpper())
	{
		_ui.plot->AddValue(3, value);
	}
}


void ControllerWindow::OnPlotSliderValueChanged(int newValue)
{
	const double SAMPLE_PERIOD_S = _txDelayMillis * 1e-3;
	const double SAMPLE_FREQ = 1 / SAMPLE_PERIOD_S;

	_ui.plot->UpdateSamplesPerSecond(SAMPLE_FREQ);

	int samplesInNewValue = SAMPLE_FREQ * newValue;

	_ui.plot->SetXSpan(samplesInNewValue);

	_ui.lblTimeSpan->setText(QString("Time: %1s").arg(newValue));
}


void ControllerWindow::OnPlotTrack1ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(0, newValue);
}


void ControllerWindow::OnPlotTrack2ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(1, newValue);
}


void ControllerWindow::OnPlotTrack3ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(2, newValue);
}


void ControllerWindow::OnPlotTrack4ValueChanged(int newValue)
{
	_ui.plot->SetYSpan(3, newValue);
}


void ControllerWindow::OnComboTrack1TextChanged(const QString& newText)
{
	//_ui.plot->SetYSpan(0, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(0);
	}
}


void ControllerWindow::OnComboTrack2TextChanged(const QString& newText)
{
	//_ui.plot->SetYSpan(1, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(1);
	}
}


void ControllerWindow::OnComboTrack3TextChanged(const QString& newText)
{
	//_ui.plot->SetYSpan(2, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(2);
	}
}


void ControllerWindow::OnComboTrack4TextChanged(const QString& newText)
{
	//_ui.plot->SetYSpan(3, _defaultPlotSpan[newText]);

	if (newText.toUpper().contains("NONE"))
	{
		_ui.plot->ClearData(3);
	}
}