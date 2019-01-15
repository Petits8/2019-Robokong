/*
 * DriverControl.h
 *
 *  Created on: Dec 6, 2018
 *      Author: Jesus Velarde
 */

#ifndef SRC_DRIVERCONTROL_H_
#define SRC_DRIVERCONTROL_H_
#define L_STICK 0
#define R_STICK 1
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

#include <Joystick.h>

class DriverControl {
private:
	frc::Joystick l_joystick{0};
	frc::Joystick r_joystick{1};
	bool bJoystick;
	double controllerVector[2][3];
	bool buttons[2][11];
public:
	DriverControl(bool bJoystick);
	bool IsJoystick();
	double GetVectorValue(int axis);
	double GetLiftValue();
	bool GetButtonValue(int stick, int button);

};

#endif /* SRC_DRIVERCONTROL_H_ */
