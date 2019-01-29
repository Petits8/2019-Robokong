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
#define SPEED_DIVIDE 2

#include <Joystick.h>
#include <DoubleSolenoid.h>
#include <DriverStation.h>

class DriverControl {
private:
	frc::Joystick l_joystick{0};
	frc::Joystick r_joystick{1};
	bool bJoystick;
	double controllerVector[2][3];
	bool buttons[2][11];
	double divider = 1;
	frc::DoubleSolenoid clawPiston{0, 1};
public:
	DriverControl(bool bJoystick);
	bool IsJoystick();
	double GetVectorValue(int axis);
	double GetLiftValue();
	bool GetButtonValue(int stick, int button);
	void Update();
	bool isFullSpeed();
	void ToggleClaw();

};

#endif /* SRC_DRIVERCONTROL_H_ */
