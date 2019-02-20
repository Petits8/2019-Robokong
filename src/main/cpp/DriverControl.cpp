/*
 * DriverControl.cpp
 *
 *  Created on: Dec 6, 2018
 *      Author: Jesus Velarde
 */

#include <DriverControl.h>

DriverControl::DriverControl(bool bJoystick){
	this->bJoystick = bJoystick;
	for(int i=0; i<2; ++i){
		for(int j=0; j<11; ++j){
			this->buttons[i][j] = false;
		}
	}
	for(int i=0; i<2; ++i){
		for(int j=0; j<3; ++j){
			this->controllerVector[i][j] = 0.0;
		}
	}
	this->timer = new frc::Timer();

	armEncoder = new frc::Encoder(1, 0, true, frc::Encoder::EncodingType::k4X);
	armEncoder->SetMaxPeriod(.1);
	armEncoder->SetMinRate(10);
	armEncoder->SetDistancePerPulse((2*PI) / 600);
	armEncoder->SetReverseDirection(true);
	armEncoder->SetSamplesToAverage(7);
	armEncoder->Reset();
}
bool DriverControl::getStationButton(int id){
	return this->d_station_controller.GetRawButton(id);
}
bool DriverControl::IsJoystick(){return this->bJoystick;}
double DriverControl::GetVectorValue(int axis){
	double Y_AXIS_VALUE;
	double Y_AXIS_VALUE_NEW;
	if(bJoystick){
		switch(axis){

		case Y_AXIS:
			return this->l_joystick.GetY()/this->divider;
			//Y_AXIS_VALUE = this->l_joystick.GetY()/this->divider;
		case X_AXIS:
			return this->r_joystick.GetX()/this->divider;
		case Z_AXIS:
			return (this->l_joystick.GetZ()+this->r_joystick.GetZ())/2;
		default:
			return 0.0;

		};
	} else {
			switch(axis){

		case Y_AXIS:


			Y_AXIS_VALUE = this->l_joystick.GetRawAxis(1)/this->divider;
			break;


		case X_AXIS:
			return this->l_joystick.GetRawAxis(4)/this->divider;


		case Z_AXIS:
		
			return 0.0;
		default:
			return 0.0;

		};

		double timeDelta = timer->Get() - timeInitial;
		timeInitial = timer->Get();

		if(timeDelta == 0.0){
			return Y_AXIS_VALUE;
		}

		yDelta = Y_AXIS_VALUE - yInitial;
		if(yDelta/timeDelta > PWRVEL){
			Y_AXIS_VALUE_NEW = yInitial + PWRVEL*timeDelta;
		} else if(yDelta/timeDelta < -PWRVEL){
			Y_AXIS_VALUE_NEW = yInitial - PWRVEL*timeDelta;
		} else{
			Y_AXIS_VALUE_NEW = Y_AXIS_VALUE;
		}

		timer->Reset();
		timer->Start();
		yInitial = Y_AXIS_VALUE_NEW;
		return Y_AXIS_VALUE_NEW;

	}
}
double DriverControl::GetLiftValue(){
	return this->r_joystick.GetY();
}

bool DriverControl::GetButtonValue(int stick, int button){
	if(stick == L_STICK){
		return this->l_joystick.GetRawButton(button);
	} else if(stick == R_STICK){
		return this->r_joystick.GetRawButton(button);
	}
	return false;
}

void DriverControl::Update(){
	
	if(this->bJoystick){
		
		//Checking Slow Down Button
		if(this->buttons[1][2] == false && this->r_joystick.GetRawButton(3)){
			if(this->divider == 1) this->divider = SPEED_DIVIDE;
			else if(this->divider == SPEED_DIVIDE) this->divider = 1;
		} 

		//Checks if R_Trigger is pressed and toggles pneumatic claw
		if(this->buttons[1][0] == false && this->r_joystick.GetRawButton(1)){
			ToggleClaw();
			frc::DriverStation::ReportError("BUTTON");
		}

		//Updating Array
		for(int i=0; i<2; i++){
			for(int j=0; j<11; j++){
				if(i==0){
					this->buttons[i][j] = this->l_joystick.GetRawButton(j+1);
				} else if(i==1){
					this->buttons[i][j] = this->r_joystick.GetRawButton(j+1);
				}
			}
		}
	}
}

bool DriverControl::isFullSpeed(){
	if(this->divider == 1) return true;
	return false;
}

void DriverControl::ToggleClaw(){
	if(this->clawPiston.Get() == frc::DoubleSolenoid::Value::kForward){
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kReverse);
	} else if(this->clawPiston.Get() == frc::DoubleSolenoid::Value::kReverse){
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
	} else{
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
	}
}

frc::Encoder* DriverControl::getArmEncoder(){
	return this->armEncoder;
}
double DriverControl::getArmEncoderInitial(){
	return this->armEncoderInitial;
}