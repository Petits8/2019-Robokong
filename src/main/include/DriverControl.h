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

	void Stay(int spot);
	void Move(double vector);
	void Goto(int target, int spot);
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
	

public:
	Wrist *wrist;
	Arm(int x, int y, int wrist_id){
		_a1 = new frc::Spark(x);
		_a2 = new frc::Spark(y);
		_arm=new frc::SpeedControllerGroup(*_a1,*_a2);
		wrist = new Wrist(wrist_id);
	}
	
	void Stay(int spot);
	void Move(double vector);
	void Goto(int target, int spot, double w_target);
	Wrist* GetWrist();
};




#endif /* SRC_DRIVERCONTROL_H_ */
