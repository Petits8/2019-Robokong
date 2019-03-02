/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include "EncoderPair.h"

#include <frc/Drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/SampleRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/Spark.h>

using namespace frc;
using namespace cs;

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
class Robot : public frc::SampleRobot {
public:
	Robot();

	void RobotInit() override;
	void Autonomous() override;
	void OperatorControl() override;
	void Test() override;

private:
	// Robot drive system

	UsbCamera m_cameraOne;
	UsbCamera m_cameraTwo;
	VideoSink m_videoSink;
	CameraServer m_cameraServer;



	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
};
