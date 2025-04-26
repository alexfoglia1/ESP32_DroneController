#ifndef JOYSTICK_BRIDGE_H
#define JOYSTICK_BRIDGE_H
#include <QObject>
#include <QJoysticks.h>

class JoystickBridge : public QObject
{
	Q_OBJECT
public:
	JoystickBridge(QObject* parent = nullptr);

signals:
	void joystickCommand(quint8 throttle, float roll, float pitch);

private:
	QJoysticks* _js;
	
	quint8 _throttle;
	float _roll;
	float _pitch;

	qreal deadCenterZone(qreal axisValue, qreal deadCenter, qreal dczValue, qreal minAxisValue, qreal maxAxisValue);
	qreal mapValue(qreal value, qreal fromMin, qreal fromMax, qreal toMin, qreal toMax);
	qreal saturate(qreal value, qreal min, qreal max);


private slots:
	void onAxisEvent(const QJoystickAxisEvent& evt);
};

#endif