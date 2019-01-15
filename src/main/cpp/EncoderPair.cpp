/*
 * EncoderPair.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: Jesus Velarde
 */

#include <EncoderPair.h>
#include <stdio.h>

EncoderPair::EncoderPair(int pinR1, int pinR2, int pinL1, int pinL2){
	rightEncoder = new frc::Encoder(pinR1, pinR2, true, frc::Encoder::EncodingType::k4X);
	rightEncoder->SetMaxPeriod(.1);
	rightEncoder->SetMinRate(10);
	rightEncoder->SetDistancePerPulse(5);
	rightEncoder->SetReverseDirection(true);
	rightEncoder->SetSamplesToAverage(7);
	rightEncoder->Reset();

	leftEncoder = new frc::Encoder(pinL1, pinL2, false, frc::Encoder::EncodingType::k4X);
	leftEncoder->SetMaxPeriod(.1);
	leftEncoder->SetMinRate(10);
	leftEncoder->SetDistancePerPulse(5);
	leftEncoder->SetReverseDirection(false);
	leftEncoder->SetSamplesToAverage(7);
	leftEncoder->Reset();

	timer = new frc::Timer();
	rightPositionDelta = leftPositionDelta = rightPositionInitial = leftPositionInitial = rightVelocity = leftVelocity = 0.0;

}

void EncoderPair::Update(){

	rightPositionDelta = rightEncoder->GetDistance() - rightPositionInitial;
	leftPositionDelta = leftEncoder->GetDistance() - leftPositionInitial;
	rightPositionInitial = rightEncoder->GetDistance();
	leftPositionInitial = leftEncoder->GetDistance();

	double timeFinal = timer->Get();
	double timeDelta = timeFinal - timeInitial;

	if(timeDelta > 0.0){
		char buf[1024];
		buf[0] = 0;
		rightVelocity = rightPositionDelta/timeDelta;
		leftVelocity = leftPositionDelta/timeDelta;

		//sprintf(buf, "R: %i - L: %i", rightEncoder->Get(), leftEncoder->Get());
		sprintf(buf, "R: %i, L: %i", rightEncoder->GetRaw(), leftEncoder->GetRaw());


		frc::DriverStation::ReportError(buf);

	} else{
		frc::DriverStation::ReportError("Time Zero Error");
	}

	timer->Reset();
	timer->Start();
	return;
}
