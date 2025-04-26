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

	const int _txDelayMillis = 100;
	
	void autoScanComPorts();
	void checkPlot(QString expectedText, double value);

private slots:
	void OnJoystickCommand(quint8 throttle, float roll, float pitch);

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
};

#endif
