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

#define LIFTMOTOR 4

frc::Spark LeftFrontMotor(0);
frc::Spark LeftRearMotor(1);
frc::Spark RightFrontMotor(2);
frc::Spark RightRearMotor(3);

frc::SpeedControllerGroup LeftMotors(LeftFrontMotor, LeftRearMotor);
frc::SpeedControllerGroup RightMotors(RightFrontMotor, RightRearMotor);

frc::DifferentialDrive m_robotDrive{LeftMotors, RightMotors};

EncoderPair *pEncoderPair = new EncoderPair(4, 5, 2, 3);
DriverControl *pDriverControl = new DriverControl(false);

frc::Spark *Lift = new frc::Spark(LIFTMOTOR);


Robot::Robot() {
	// Note SmartDashboard is not initialized here, wait until
	// RobotInit to make SmartDashboard calls
	m_robotDrive.SetExpiration(0.1);
}

void Robot::RobotInit() {
	m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  	frc::CameraServer::GetInstance()->StartAutomaticCapture(1);

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
	char buf[1024];
	buf[0] = 0;
	m_robotDrive.SetSafetyEnabled(true);
	while (IsOperatorControl() && IsEnabled()) {

		// Drive with arcade style (use right stick)
		m_robotDrive.ArcadeDrive(pDriverControl->GetVectorValue(Y_AXIS), pDriverControl->GetVectorValue(X_AXIS));
		//sprintf(buf, "Y: %f - X: %f", -pDriverControl->GetVectorValue(Y_AXIS), pDriverControl->GetVectorValue(X_AXIS));

		//Lift->Set(-pDriverControl->GetLiftValue());


		pEncoderPair->Update();

		// The motors will be updated every 5ms
		frc::DriverStation::ReportError(buf);
		frc::Wait(0.005);
	}
}

void DriveStraight(){

}

/**
 * Runs during test mode
 */
void Robot::Test() {}

START_ROBOT_CLASS(Robot)
