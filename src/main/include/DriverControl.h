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
#define DRVFORWARD 0
#define DRVROTATE 1
#define DRVARM 3
#define DRVWRIST 4
#define SPEED_DIVIDE 2

#include <Joystick.h>
#include <DoubleSolenoid.h>
#include <DriverStation.h>
#include <Timer.h>
#include <SpeedControllerGroup.h>
#include <Spark.h>
#include <rev/SparkMax.h>
#include <rev/CANSparkMax.h>


#define PWRVEL 5


#define LOW_CUTOFF 75				// Below this angle, use these parameters
#define LOW_WRIST_SPEED .07			// Speed the wrist will travel
#define LOW_WRIST_HOLD .03 			// Power required to hold wrist in place
#define LOW_ARM_UP .22				// Slow speed for arm going "up" or away from starting point
#define LOW_ARM_DOWN .2				// Slow speed for arm going "down" or towards starting point
#define	LOW_ARM_HOLD 0				// Power required to hold the arm in place
#define LOW_MID_SPEED .35			// Mid speed
#define LOW_HIGH_SPEED .5			// High speed
#define LOW_MID_CUTOFF 10			// Cutoff encoder reading for Mid Speed
#define LOW_HIGH_CUTOFF 15			// Cutoff encoder reading for High Speed

#define MID_CUTOFF 150
#define MID_WRIST_SPEED .07
#define MID_WRIST_HOLD .03 
#define MID_ARM_UP .35
#define MID_ARM_DOWN .2
#define	MID_ARM_HOLD .03
#define MID_MID_SPEED .5
#define MID_HIGH_SPEED .75
#define MID_MID_CUTOFF 10
#define MID_HIGH_CUTOFF 15

#define HIGH_CUTOFF 180
#define HIGH_WRIST_SPEED .1
#define HIGH_WRIST_HOLD .03 
#define HIGH_ARM_UP .3
#define HIGH_ARM_DOWN .05
#define	HIGH_ARM_HOLD .0
#define HIGH_MID_SPEED .35
#define HIGH_HIGH_SPEED .75
#define HIGH_MID_CUTOFF 10
#define HIGH_HIGH_CUTOFF 15

#define GRAB_CUTOFF 490
#define GRAB_WRIST_SPEED .1
#define GRAB_WRIST_HOLD .0 
#define GRAB_ARM_UP .2
#define GRAB_ARM_DOWN .05
#define	GRAB_ARM_HOLD .2
#define GRAB_MID_SPEED .25
#define GRAB_HIGH_SPEED .75
#define GRAB_MID_CUTOFF 30
#define GRAB_HIGH_CUTOFF 50

class DriverControl {
private:
	frc::Joystick l_joystick{0};
	frc::Joystick r_joystick{1};
	frc::Joystick d_station_controller{2};
	bool bJoystick;
	double controllerVector[2][3];
	bool buttons[2][11];
	double divider = 1;
	frc::DoubleSolenoid clawPiston{0, 1};
	frc::Timer *timer;
	double yInitial, timeInitial, yDelta = 0.0;
	
public:
	DriverControl(bool bJoystick);
	bool IsJoystick();
	double GetVectorValue(int axis);
	double GetLiftValue();
	bool GetButtonValue(int stick, int button);
	void Update();
	bool isFullSpeed();
	void ToggleClaw();
	bool getStationButton(int id);
	int getStationButton();
};	

class Wrist {
private:
	rev::CANSparkMax *_wrist;
	char mybuf[1024];
	double encoderInitial;

public:
	Wrist(int id){
		_wrist = new rev::CANSparkMax(id, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
			char buf[1024];
		
	sprintf(buf, "Wrist init");
	frc::DriverStation::ReportError(buf);

	}
	void Init();

	void Tuck(int target, int spot);
	void Move(double vector);
	void Goto(int target, int spot, int ArmPosition);
	void Update();
	int Get();
	void Zero();
	double GetEncoderValue();
};

class Arm {
private:
	frc::SpeedControllerGroup *_arm;
	frc::Spark *_a1;
	frc::Spark *_a2;
	frc::Timer *timer;
	

public:
	Wrist *wrist;
	Arm(int x, int y, int wrist_id){
		_a1 = new frc::Spark(x);
		_a2 = new frc::Spark(y);
		_arm=new frc::SpeedControllerGroup(*_a1,*_a2);
		wrist = new Wrist(wrist_id);
		this->timer = new frc::Timer();
	}
	
	void Stay(int spot);
	void Move(double vector);
	void Goto(int target, int spot, double w_target);
	Wrist* GetWrist();
};




#endif /* SRC_DRIVERCONTROL_H_ */
