/*
 * EncoderPair.h
 *
 *  Created on: Dec 5, 2018
 *      Author: Jesus Velarde
 */

#ifndef SRC_ENCODERPAIR_H_
#define SRC_ENCODERPAIR_H_

#include "Robot.h"
#include <Encoder.h>
#include <DigitalInput.h>
#include <Timer.h>
#include <DriverStation.h>
#include <math.h>

class EncoderPair {
private:
	frc::Encoder *rightEncoder, *leftEncoder;
	frc::Timer *timer;
	double timeInitial = 0.0;
	double rightPositionDelta, leftPositionDelta, rightPositionInitial, leftPositionInitial, rightVelocity, leftVelocity;
	int cycle = 0;

public:
	frc::Encoder getRightEncoder();
	frc::Encoder getLeftEncoder();
	void Update();
	double getDeltaVelocity();
	double getRightEncoderVelocity();
	double getLeftEncoderVelocity();

	EncoderPair(int pinR1, int pinR2, int pinL1, int pinL2);
};

#endif /* SRC_ENCODERPAIR_H_ */
