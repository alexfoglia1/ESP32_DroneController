#ifndef CONTROLLERWINDOW_H
#define CONTROLLERWINDOW_H

#include <qmainwindow.h>
#include <QJoysticks.h>

#include "JoystickBridge.h"
#include "SerialComm.h"
#include "UdpComm.h"

#include "ui_ControllerGui.h"
#include "ui_AutoscanComPortsGui.h"


class ControllerWindow : public QMainWindow
{
	Q_OBJECT
public:
	ControllerWindow();

private:
	QMainWindow _autoscanProgressWindow;
	Ui_MainWindow _ui;
	Ui_AutoscanComPortsGui _progressUi;
	JoystickBridge _js;
	SerialComm _serialComm;
	UdpComm _udpComm;
	float _lastRoll;
	float _lastPitch;
	bool _synchOnNextAttitude;
	bool _attitudeSynch;
	quint8 _lastThrottle;

	const int _txDelayMillis = 100;
	
	void autoScanComPorts();
	void checkPlot(QString expectedText, double value);

private slots:
	void OnJoystickCommand(quint8 throttle, float roll, float pitch);
	void OnArmedCommand(bool isArmed);

	void OnBtnOpenSerialPort();
	void OnBtnRescanPorts();
	void OnBtnConnect();
	
	void OnPlotSliderValueChanged(int newValue);
	void OnPlotTrack1ValueChanged(int newValue);
	void OnPlotTrack2ValueChanged(int newValue);
	void OnPlotTrack3ValueChanged(int newValue);
	void OnPlotTrack4ValueChanged(int newValue);
	void OnComboTrack1TextChanged(const QString& newText);
	void OnComboTrack2TextChanged(const QString& newText);
	void OnComboTrack3TextChanged(const QString& newText);
	void OnComboTrack4TextChanged(const QString& newText);
	void OnCheckGetAttitude();
	void OnCheckGetAcc();
	void OnCheckGetGyro();
	void OnCheckGetRollPid();
	void OnCheckGetPitchPid();
	void OnCheckGetStatus();
	void OnCheckJoySynchAttitude();
	void OnBtnResetJoySynch();
	void OnBtnTestMotors();
	void OnBtnSendRollPid();
	void OnBtnSendPitchPid();

};

#endif
