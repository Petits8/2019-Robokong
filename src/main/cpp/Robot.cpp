/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <SmartDashboard/SmartDashboard.h>
#include <Timer.h>
#include <Spark.h>
#include <SpeedControllerGroup.h>
#include <Drive/DifferentialDrive.h>
#include <DriverControl.h>
#include <CameraServer.h>
#include <VisionProcessing.h>
#include <CAN.h>

#define LIFTMOTOR 4
#define BGRAB 8
#define BTOP 7
#define BCENTER 6
#define BLOW 5



frc::Spark MRight_0(0);
frc::Spark MRight_1(1);
frc::Spark MRight_2(2);
frc::Spark MLeft_0(3);
frc::Spark MLeft_1(4);
frc::Spark MLeft_2(5);

//frc::Spark M_ARM1(6);
//frc::Spark M_ARM2(7);

frc::Spark M_INTAKEROLLER(8);
frc::Spark M_INTAKEARM(9);



//frc::SpeedControllerGroup MCG_ARM(M_ARM1, M_ARM2);
frc::SpeedControllerGroup RightMotors(MRight_0, MRight_1, MRight_2);
frc::SpeedControllerGroup LeftMotors(MLeft_0, MLeft_1, MLeft_2);


//frc::Spark YawCameraController(5);
//frc::Spark PitchCameraController(6);

frc::DifferentialDrive m_robotDrive{LeftMotors, RightMotors};

//EncoderPair *pEncoderPair = new EncoderPair(4, 5, 2, 3);
EncoderSingle *pShoulderEncoder = new EncoderSingle(0,1);

DriverControl *pDriverControl = new DriverControl(true);
Arm *arm = new Arm(6,7,2);


//frc::Spark *Lift = new frc::Spark(LIFTMOTOR);


Robot::Robot() {
	// Note SmartDashboard is not initialized here, wait until
	// RobotInit to make SmartDashboard calls
	m_robotDrive.SetExpiration(0.1);
}

void Robot::RobotInit() {
	//Vision Processing will be done on the Raspberry PI 2
	//m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	//m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	

	//std::thread visionThread(VisionThread);
    //visionThread.detach();

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::Autonomous() {
	std::string autoSelected = m_chooser.GetSelected();
	// std::string autoSelected = frc::SmartDashboard::GetString(
	// "Auto Selector", kAutoNameDefault);
	std::cout << "Auto selected: " << autoSelected << std::endl;

	// MotorSafety improves safety when motors are updated in loops
	// but is disabled here because motor updates are not looped in
	// this autonomous mode.
	m_robotDrive.SetSafetyEnabled(false);

	m_robotDrive.ArcadeDrive(1, 1);

}

/**
 * Runs the motors with arcade steering.
 */
void Robot::OperatorControl() {
	bool bInitLatch = false; 
	char buf[1024];
	buf[0] = 0;
	m_robotDrive.SetSafetyEnabled(true);
	int b7Latch=0;
	bool bButtonReleaseLatch = true; 
	float w_last;
	int arm_last;
	int roller_state = 0;
	int d_joystick_latch = 0;
	
	while (IsOperatorControl() && IsEnabled()) {
		if (!bInitLatch){
			arm->GetWrist()->Init();
			pShoulderEncoder->Zero();
			bInitLatch = true; 
		}
		pShoulderEncoder->Update();
		pDriverControl->Update();
			
		// Drive with arcade style (use right stick)

		

		switch (pDriverControl->getStationButton()) {
			case BGRAB:
				arm->Goto(STOP_GRAB, pShoulderEncoder->Get(), WRIST_GRAB);
				bButtonReleaseLatch = false;
				break;
			case BTOP:
				arm->Goto(STOP_TOP, pShoulderEncoder->Get(), WRIST_TOP);
				bButtonReleaseLatch = false;
				break;
			case BCENTER:
				arm->Goto(STOP_CENTER, pShoulderEncoder->Get(), WRIST_CENTER);
				bButtonReleaseLatch = false;
				break;
			case BLOW:
				arm->Goto(STOP_LOW, pShoulderEncoder->Get(), WRIST_LOW);
				bButtonReleaseLatch = false;
				break;
			case 10:
				
				if(abs(pDriverControl->d_station_controller.GetX()) > .1 || abs(pDriverControl->d_station_controller.GetY()) > .1){
					arm->Move(pDriverControl->d_station_controller.GetX() * .5);
					arm->GetWrist()->Move(pDriverControl->d_station_controller.GetY() * .1);
					bButtonReleaseLatch = false;
				} else{
						if(!bButtonReleaseLatch){
						w_last = arm->GetWrist()->GetEncoderValue();
						arm_last = pShoulderEncoder->Get();
						bButtonReleaseLatch = true;
						
						}
					arm->Stay(arm_last, pShoulderEncoder->Get());
					arm->GetWrist()->Stay(w_last);
				}
				break;
			default:
				//arm->Move(pDriverControl->GetVectorValue(DRVARM));
				//arm->GetWrist()->Move(pDriverControl->GetVectorValue(X_AXIS));
				if(!bButtonReleaseLatch){
					w_last = arm->GetWrist()->GetEncoderValue();
					arm_last = pShoulderEncoder->Get();
					bButtonReleaseLatch = true;
				}
				arm->Stay(arm_last, pShoulderEncoder->Get());
				arm->GetWrist()->Stay(w_last);

				M_INTAKEARM.Set(pDriverControl->d_station_controller.GetX() * -.7);
		
				double d_joy_y = pDriverControl->d_station_controller.GetY();
				if(d_joy_y == 1.0 && d_joystick_latch != 1){
					if(roller_state < 1){
						roller_state += 1;
					}
					d_joystick_latch = 1;
				} else if(d_joy_y == -1.0 && d_joystick_latch != -1){
					if(roller_state > -1){
						roller_state -= 1;
					}
					d_joystick_latch = -1;
				} else if(d_joy_y != 1.0 && d_joy_y != -1.0){
					d_joystick_latch = 0;
				}
				
				M_INTAKEROLLER.Set(roller_state * .7);


				break;
		}

		//arm->GetWrist()->Move(pDriverControl->GetVectorValue(DRVWRIST));
		
		

		//sprintf(buf, "INTAKE ARM: %f", pDriverControl->d_station_controller.GetX());
		//frc::DriverStation::ReportError(buf);
		
		//m_robotDrive.ArcadeDrive(0, 0, pDriverControl->isFullSpeed());
		m_robotDrive.ArcadeDrive(pDriverControl->GetVectorValue(X_AXIS), pDriverControl->GetVectorValue(Y_AXIS), pDriverControl->isFullSpeed());
//		sprintf(buf, "Y: %f - X: %f", -pDriverControl->GetVectorValue(Y_AXIS), pDriverControl->GetVectorValue(X_AXIS));
//		sprintf(buf, "%d", pDriverControl->getStationButton(3));
		//sprintf(buf, "Shoulder: %d %d", pShoulderEncoder->Get(), pDriverControl->getStationButton());
		if(pDriverControl->GetButtonValue(R_STICK, 11)){
			 sprintf(buf, "WRIST: %f : ARM: %i", arm->GetWrist()->GetEncoderValue(), pShoulderEncoder->Get());
			 frc::DriverStation::ReportError(buf);
		}

		//Lift->Set(-pDriverControl->GetLiftValue());

		//YawCameraController.Set(pDriverControl->GetVectorValue(X_AXIS));
		//PitchCameraController.Set(pDriverControl->GetVectorValue(Y_AXIS));
		

		// The motors will be updated every 5ms
		
		frc::Wait(0.005);
	}
	bInitLatch = false;
}

void DriveStraight(){

}

/**
 * Runs during test mode
 */
void Robot::Test() {}

START_ROBOT_CLASS(Robot)
