/*
 * DriverControl.cpp
 *
 *  Created on: Dec 6, 2018
 *      Author: Jesus Velarde
 */

#include <DriverControl.h>
#include <math.h>

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
}
bool DriverControl::getStationButton(int id){
	return this->d_station_controller.GetRawButton(id);
}
int DriverControl::getStationButton(){
	if (this->getStationButton(1)) return 1;
	if (this->getStationButton(2)) return 2;
	if (this->getStationButton(3)) return 3;
	if (this->getStationButton(4)) return 4;
	if (this->getStationButton(5)) return 5;
	if (this->getStationButton(6)) return 6;
	if (this->getStationButton(7)) return 7;
	if (this->getStationButton(8)) return 8;
	if (this->getStationButton(9)) return 9;
	if (this->getStationButton(10)) return 10;
}

bool DriverControl::IsJoystick(){return this->bJoystick;}
double DriverControl::GetVectorValue(int axis){
	double Y_AXIS_VALUE;
	double Y_AXIS_VALUE_NEW;
	if(bJoystick){
		switch(axis){

		case DRVFORWARD:
			return this->l_joystick.GetY()/this->divider;
			break;
			//Y_AXIS_VALUE = this->l_joystick.GetY()/this->divider;
		case DRVROTATE:
			return this->r_joystick.GetX()/this->divider;
			break;
		case Z_AXIS:
			return (this->l_joystick.GetZ()+this->r_joystick.GetZ())/2;
			break;
		case DRVARM:
			return this->l_joystick.GetX()/this->divider;
			break;
		case DRVWRIST:
			return this->r_joystick.GetRawAxis(1)/this->divider;	
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
	} else if(frc::DoubleSolenoid::Value::kReverse){
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
	} else{
		this->clawPiston.Set(frc::DoubleSolenoid::Value::kForward);
	}
}


void Arm::Stay(int spot){

}

void Arm::Move(double vector){
	this->_arm->Set(vector);
}

void Arm::Goto(int target, int spot, double w_target) {	
	char buf[1024];
	int error;
	double speed;


	speed=0;
	error=abs(spot-target);
	if (error >=0 && error < 2) { 
		speed=.1; 
		sprintf(buf, "<2 %f ",speed);

		this->GetWrist()->Goto(w_target, this->GetWrist()->GetEncoderValue());


	} else{
		this->GetWrist()->Move(0);
	}
	if (error >= 2 && error < 5) { 
		speed=.35; 
		sprintf(buf, ">=2 %f ",speed);
	}
	if (error >= 5) {
		speed=.5; 
		sprintf(buf, ">=5 %f ",speed);
	}

	sprintf(buf, "%s Goto: T:%d S:%d Sp: %f E:%d", buf, target, spot, speed, error);
	frc::DriverStation::ReportError(buf);

	if (target < spot) {
		this->_arm->Set(speed);
	} else if (target > spot){
		this->_arm->Set(-speed);
	} else {
		this->_arm->Set(-speed);
	}
}	

void Wrist::Stay(int spot){

}

void Wrist::Move(double vector){
	char buf[1024];

	this->_wrist->Set(vector);
	//sprintf(buf, "Wrist: %f : %f", vector, this->_wrist->Get());
	//frc::DriverStation::ReportError(buf);
}

void Wrist::Goto(int target, int spot){
	double speed = .4;
	double error = target-spot;
	if(abs(error) < .75){ 
		speed = .1;
	}
	if (target < spot) {
		this->Move(-speed);
	} else if (target > spot){
		this->Move(speed);
	} else {
		this->Move(speed);
	}
}

void Wrist::Update(){

}

int Wrist::Get(){

}

void Wrist::Zero(){

}
void Wrist::Init(){
	this->encoderInitial = _wrist->GetEncoder().GetPosition();
}

double Wrist::GetEncoderValue(){
	return this->_wrist->GetEncoder().GetPosition() - this->encoderInitial;
}
Wrist* Arm::GetWrist(){
	return this->wrist;
}