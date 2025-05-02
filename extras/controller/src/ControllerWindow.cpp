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
	connect(&_js, SIGNAL(armedCommand(bool)), this, SLOT(OnArmedCommand(bool)));

	connect(_ui.btnOpenSerialPort, SIGNAL(clicked()), this, SLOT(OnBtnOpenSerialPort()));
	connect(_ui.btnRescanPorts, SIGNAL(clicked()), this, SLOT(OnBtnRescanPorts()));
	connect(_ui.btnConnect, SIGNAL(clicked()), this, SLOT(OnBtnConnect()));
	connect(_ui.btnResetAttitudeSync, SIGNAL(clicked()), this, SLOT(OnBtnResetJoySynch()));
	connect(_ui.btnSendTestMotors, SIGNAL(clicked()), this, SLOT(OnBtnTestMotors()));
	connect(_ui.btnSendRollPid, SIGNAL(clicked()), this, SLOT(OnBtnSendRollPid()));
	connect(_ui.btnSendPitchPid, SIGNAL(clicked()), this, SLOT(OnBtnSendPitchPid()));

	connect(_ui.plotTimeSlider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotSliderValueChanged(int)));
	connect(_ui.plotTrack1Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack1ValueChanged(int)));
	connect(_ui.plotTrack2Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack2ValueChanged(int)));
	connect(_ui.plotTrack3Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack3ValueChanged(int)));
	connect(_ui.plotTrack4Slider, SIGNAL(valueChanged(int)), this, SLOT(OnPlotTrack4ValueChanged(int)));

	connect(_ui.comboSelTrack1, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack1TextChanged(const QString&)));
	connect(_ui.comboSelTrack2, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack2TextChanged(const QString&)));
	connect(_ui.comboSelTrack3, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack3TextChanged(const QString&)));
	connect(_ui.comboSelTrack4, SIGNAL(currentTextChanged(const QString&)), this, SLOT(OnComboTrack4TextChanged(const QString&)));

	connect(_ui.checkGetAttitude, SIGNAL(clicked()), this, SLOT(OnCheckGetAttitude()));
	connect(_ui.checkGetAcc, SIGNAL(clicked()), this, SLOT(OnCheckGetAcc()));
	connect(_ui.checkGetGyro, SIGNAL(clicked()), this, SLOT(OnCheckGetGyro()));
	connect(_ui.checkGetRollPid, SIGNAL(clicked()), this, SLOT(OnCheckGetRollPid()));
	connect(_ui.checkGetPitchPid, SIGNAL(clicked()), this, SLOT(OnCheckGetPitchPid()));
	connect(_ui.checkGetStatus, SIGNAL(clicked()), this, SLOT(OnCheckGetStatus()));

	connect(_ui.checkJoySynchAttitude, SIGNAL(clicked()), this, SLOT(OnCheckJoySynchAttitude()));

	connect(_ui.serialTerminalWidget, SIGNAL(commandSent(const QString&)), &_serialComm, SLOT(sendMessage(const QString&)));
	connect(&_serialComm, SIGNAL(messageReceived(const QString&)), _ui.serialTerminalWidget, SLOT(responseReceived(const QString&)));

	// TODO : completare udp comm con signal/slot non lambda
	connect(&_udpComm, &UdpComm::uplink, this, [this]()
		{
			_ui.lblConnStatus->setStyleSheet(("background-color: #00ff00"));
		});

	connect(&_udpComm, &UdpComm::downlink, this, [this]()
		{
			_ui.lblConnStatus->setStyleSheet(("background-color: #ff0000"));
		});

	connect(&_udpComm, &UdpComm::receivedAttitude, this, [this](float roll, float pitch, float yaw) {

		if (_synchOnNextAttitude)
		{
			_lastRoll = roll;
			_lastPitch = pitch;

			_synchOnNextAttitude = false;
			_attitudeSynch = true;
			_ui.checkJoySynchAttitude->setChecked(false);
		}

		checkPlot("BODY_ROLL", roll);
		checkPlot("BODY_PITCH", pitch);
		checkPlot("BODY_YAW", yaw);

		_ui.pfdRollPitch->UpdateRoll(roll);
		_ui.pfdRollPitch->UpdatePitch(pitch);
		_ui.pfdHeading->UpdateHeading(yaw);

		_ui.lineRoll->setText(QString::number(roll));
		_ui.linePitch->setText(QString::number(pitch));
		_ui.lineYaw->setText(QString::number(yaw));
		});

	connect(&_udpComm, &UdpComm::receivedRawAccel, this, [this](float ax, float ay, float az)
		{
			checkPlot("RAW_ACC_X", ax);
			checkPlot("RAW_ACC_Y", ay);
			checkPlot("RAW_ACC_Z", az);

			_ui.lineAccX->setText(QString::number(ax));
			_ui.lineAccY->setText(QString::number(ay));
			_ui.lineAccZ->setText(QString::number(az));
		});

	connect(&_udpComm, &UdpComm::receivedRawGyro, this, [this](float gx, float gy, float gz)
		{
			checkPlot("RAW_GYRO_X", gx);
			checkPlot("RAW_GYRO_Y", gy);
			checkPlot("RAW_GYRO_Z", gz);

			_ui.lineGyroX->setText(QString::number(gx));
			_ui.lineGyroY->setText(QString::number(gy));
			_ui.lineGyroZ->setText(QString::number(gz));
		});

	connect(&_udpComm, &UdpComm::receivedRollPid, this, [this](float P, float I, float D, float U)
		{
			checkPlot("ROLL_PID_P", P);
			checkPlot("ROLL_PID_I", I);
			checkPlot("ROLL_PID_D", D);
			checkPlot("ROLL_PID_U", U);

			_ui.lineRollPidP->setText(QString::number(P));
			_ui.lineRollPidI->setText(QString::number(I));
			_ui.lineRollPidD->setText(QString::number(D));
			_ui.lineRollPidU->setText(QString::number(U));
		});

	connect(&_udpComm, &UdpComm::receivedPitchPid, this, [this](float P, float I, float D, float U)
		{
			checkPlot("PITCH_PID_P", P);
			checkPlot("PITCH_PID_I", I);
			checkPlot("PITCH_PID_D", D);
			checkPlot("PITCH_PID_U", U);

			_ui.linePitchPidP->setText(QString::number(P));
			_ui.linePitchPidI->setText(QString::number(I));
			_ui.linePitchPidD->setText(QString::number(D));
			_ui.linePitchPidU->setText(QString::number(U));
		});

	connect(&_udpComm, &UdpComm::receivedStatus, this, [this](uint8_t M1, uint8_t M2, uint8_t M3, uint8_t M4, uint8_t throttle_sp, float roll_sp, float pitch_sp)
		{
			checkPlot("MOTOR_1", M1);
			checkPlot("MOTOR_2", M2);
			checkPlot("MOTOR_3", M3);
			checkPlot("MOTOR_4", M4);
			checkPlot("CMD_THROTTLE", throttle_sp);
			checkPlot("CMD_ROLL", roll_sp);
			checkPlot("CMD_PITCH", pitch_sp);

			_ui.lineM1->setText(QString::number(M1));
			_ui.lineM2->setText(QString::number(M2));
			_ui.lineM3->setText(QString::number(M3));
			_ui.lineM4->setText(QString::number(M4));
			_ui.lineThrottleSp->setText(QString::number(throttle_sp));
			_ui.lineRollSp->setText(QString::number(roll_sp));
			_ui.linePitchSp->setText(QString::number(pitch_sp));
		});

	_lastRoll = 0.0f;
	_lastPitch = 0.0f;
	_lastThrottle = 0;
	_synchOnNextAttitude = false;
	_attitudeSynch = false;

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
	if (_attitudeSynch)
	{
		roll = _lastRoll;
		pitch = _lastPitch;
	}

	_lastThrottle = throttle;

	_udpComm.updateCommand(throttle, roll, pitch);

	_ui.lineTxThrottleSignal->setText(QString::number(throttle));
	_ui.lineTxRollSignal->setText(QString::number(roll));
	_ui.lineTxPitchSignal->setText(QString::number(pitch));
}


void ControllerWindow::OnArmedCommand(bool isArmed)
{
	_udpComm.sendArmedCommand(isArmed);
	_ui.checkIsJoyArmed->setChecked(isArmed);
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
	if (_ui.btnConnect->text().toUpper() == "CONNECT")
	{
		QString addr = _ui.lineIpAddr->text();
		short port = _ui.linePort->text().toShort();
		_ui.linePort->setText(QString::number(port));

		_udpComm.start(addr, port);

		_ui.btnConnect->setText("Disconnect");
		_ui.lineIpAddr->setEnabled(false);
		_ui.linePort->setEnabled(false);
	}
	else
	{
		_udpComm.stop();

		_ui.btnConnect->setText("Connect");
		_ui.lineIpAddr->setEnabled(true);
		_ui.linePort->setEnabled(true);
	}
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


void ControllerWindow::OnCheckGetAttitude()
{
	_udpComm.setGetEnabled(UdpComm::GetFlag::ATTITUDE, _ui.checkGetAttitude->isChecked());
}


void ControllerWindow::OnCheckGetAcc()
{
	_udpComm.setGetEnabled(UdpComm::GetFlag::ACCEL, _ui.checkGetAcc->isChecked());
}


void ControllerWindow::OnCheckGetGyro()
{
	_udpComm.setGetEnabled(UdpComm::GetFlag::GYRO, _ui.checkGetGyro->isChecked());
}


void ControllerWindow::OnCheckGetRollPid()
{
	_udpComm.setGetEnabled(UdpComm::GetFlag::ROLL_PID, _ui.checkGetRollPid->isChecked());
}

void ControllerWindow::OnCheckGetPitchPid()
{
	_udpComm.setGetEnabled(UdpComm::GetFlag::PITCH_PID, _ui.checkGetPitchPid->isChecked());
}


void ControllerWindow::OnCheckGetStatus()
{
	_udpComm.setGetEnabled(UdpComm::GetFlag::STATUS, _ui.checkGetStatus->isChecked());
}


void ControllerWindow::OnCheckJoySynchAttitude()
{
	_synchOnNextAttitude = _ui.checkJoySynchAttitude->isChecked();
}


void ControllerWindow::OnBtnResetJoySynch()
{
	_attitudeSynch = false;
}


void ControllerWindow::OnBtnTestMotors()
{
	uint8_t motors = 0;

	if (_ui.checkTestM1->isChecked())
	{
		motors |= static_cast<uint8_t>(UdpComm::MotorFlag::MOTOR_1);
	}
	if (_ui.checkTestM2->isChecked())
	{
		motors |= static_cast<uint8_t>(UdpComm::MotorFlag::MOTOR_2);
	}
	if (_ui.checkTestM3->isChecked())
	{
		motors |= static_cast<uint8_t>(UdpComm::MotorFlag::MOTOR_3);
	}
	if (_ui.checkTestM4->isChecked())
	{
		motors |= static_cast<uint8_t>(UdpComm::MotorFlag::MOTOR_4);
	}

	_udpComm.testMotors(static_cast<UdpComm::MotorFlag>(motors), static_cast<uint8_t>(_ui.spinTestMotorsPwm->value() & 0xFF));
}


void ControllerWindow::OnBtnSendRollPid()
{
	float kp = static_cast<float>(_ui.spinRollKp->value());
	float ki = static_cast<float>(_ui.spinRollKi->value());
	float kd = static_cast<float>(_ui.spinRollKd->value());

	_udpComm.setRollPid(kp, ki, kd);
}


void ControllerWindow::OnBtnSendPitchPid()
{
	float kp = static_cast<float>(_ui.spinPitchKp->value());
	float ki = static_cast<float>(_ui.spinPitchKi->value());
	float kd = static_cast<float>(_ui.spinPitchKd->value());

	_udpComm.setPitchPid(kp, ki, kd);
}
