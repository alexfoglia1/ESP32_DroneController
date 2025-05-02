#include "JoystickBridge.h"

#ifdef __linux__
#include <cmath>
#endif

JoystickBridge::JoystickBridge(QObject* parent) : QObject(parent)
{
	_js = QJoysticks::getInstance();

	_throttle = 0.0;
	_roll = 0.0;
	_pitch = 0.0;

	_armed = false;

	connect(_js, &QJoysticks::axisEvent, this, &JoystickBridge::onAxisEvent);
	connect(_js, &QJoysticks::buttonEvent, this, &JoystickBridge::onButtonEvent);
}


qreal JoystickBridge::deadCenterZone(qreal axisValue, qreal deadCenter, qreal dczValue, qreal minAxisValue, qreal maxAxisValue)
{
	if (fabs(axisValue) < deadCenter)
	{
		return dczValue;
	}
	else
	{
		double axisSpan = maxAxisValue - minAxisValue;

		if (minAxisValue < 0)
		{
			if (axisValue < 0)
			{
				return mapValue(axisValue, minAxisValue, -deadCenter, -20, 0);
			}
			else
			{
				return mapValue(axisValue, deadCenter, maxAxisValue, 0.0, 20.0);
			}
		}
		else
		{
			return mapValue(axisValue, minAxisValue + deadCenter, maxAxisValue, 0.0, 40.0) - 20.0;
		}
	}
}


qreal JoystickBridge::saturate(qreal value, qreal min, qreal max)
{
	return value < min ? min : value > max ? max : value;
}


qreal JoystickBridge::mapValue(qreal value, qreal fromMin, qreal fromMax, qreal toMin, qreal toMax)
{
	qreal fromSpan = fromMax - fromMin;
	qreal fromPercentage = ((qreal)value - (qreal)fromMin) / (qreal)fromSpan;

	int toSpan = toMax - toMin;

	return toMin + fromPercentage * toSpan;
}



void JoystickBridge::onAxisEvent(const QJoystickAxisEvent& evt)
{
	qreal evtValue = saturate(evt.value, -1.0, 1.0);
	//printf("evt.axis(%d)\n", evt.axis);

	bool isValidAxisEvent = (evt.axis == 5 || evt.axis == 0 || evt.axis == 1);
	if (isValidAxisEvent)
	{
		quint8 throttle = _throttle;
		float roll = _roll;
		float pitch = _pitch;

		if (evt.axis == 5)
		{
			//if (evtValue < 0) evtValue = 0;
			qreal fSignal = mapValue(evtValue, 0, 1.0, 0.0, 255.0);// deadCenterZone(-evtValue, 0.1, 1500.0, -1.0, 1.0);
			//printf("evtValue(%f)\tfsignal(%f)\n", evtValue, fSignal);
			_throttle = (quint8)(fSignal);
		}
		else if (evt.axis == 0)
		{
			qreal fSignal = deadCenterZone(evtValue, 0.2, 0.0, -1.0, 1.0);
			_roll = (float)(fSignal);
		}
		else if (evt.axis == 1)
		{
			qreal fSignal = deadCenterZone(-evtValue, 0.2, 0.0, -1.0, 1.0);
			_pitch = (float)(fSignal);
		}

		bool axisUpdate = ((throttle != _throttle) || (roll != _roll) || (pitch != _pitch));
		if (axisUpdate)
		{
			emit joystickCommand(_throttle, _roll, _pitch);
		}

	}
}



void JoystickBridge::onButtonEvent(const QJoystickButtonEvent& evt)
{
	if (evt.button == 0 && evt.pressed)
	{
		_armed = !_armed;
		emit armedCommand(_armed);
	}
}