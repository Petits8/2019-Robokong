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
}
bool DriverControl::IsJoystick(){return this->bJoystick;}
double DriverControl::GetVectorValue(int axis){
	if(bJoystick){
		switch(axis){

		case Y_AXIS:
			return this->l_joystick.GetY()/this->divider;
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


			return this->l_joystick.GetRawAxis(1)/this->divider;


		case X_AXIS:
			return this->l_joystick.GetRawAxis(4)/this->divider;


		case Z_AXIS:
		
			return 0.0;
		default:
			return 0.0;

		};
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
