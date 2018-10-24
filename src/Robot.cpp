#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"
#include "Functions.h"
//#include "AHRS.h"
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace nt;
using namespace std;
class Robot: public IterativeRobot {
private:

	double reverse = -1;
	double armPos = 0;

	// We get arm rate from driver station
	double armRate;

	// PID for the arm
	double p = .001, i = .001, d = 1;

	TalonSRX * lDrive1 = new TalonSRX(8);
	TalonSRX * lDrive2 = new TalonSRX(11);
	TalonSRX * rDrive1 = new TalonSRX(7);
	TalonSRX * rDrive2 = new TalonSRX(14);
	TalonSRX * lIntake = new TalonSRX(3);
	TalonSRX * rIntake = new TalonSRX(2);
	TalonSRX * arm = new TalonSRX(9);
	DoubleSolenoid * intakes = new DoubleSolenoid(0, 1);
	Joystick * dr = new Joystick(0);
	Joystick * op = new Joystick(1);

	// Add an object for the limit switch
	int limitSwitchPort = 9; // Limit switch port is number 9 (by the RSL)
	DigitalInput * limitSwitch = new DigitalInput(limitSwitchPort);

	PowerDistributionPanel * PDP = new PowerDistributionPanel();

	void RobotInit() {

		// Initalize the preference instance object thingy idk what im talking about
		SmartDashboard::PutNumber("Arm p", p);
		SmartDashboard::PutNumber("Arm i", i);
		SmartDashboard::PutNumber("Arm d", d);
		SmartDashboard::PutNumber("Arm rate", armRate);
		// Eat my ass

		rDrive2->Set(ControlMode::Follower, 7);
		lDrive2->Set(ControlMode::Follower, 8);
		lDrive1->ConfigContinuousCurrentLimit(30, kTimeoutMs);
		lDrive2->ConfigContinuousCurrentLimit(30, kTimeoutMs);
		rDrive1->ConfigContinuousCurrentLimit(30, kTimeoutMs);
		rDrive2->ConfigContinuousCurrentLimit(30, kTimeoutMs);
		rDrive1->SetInverted(true);
		rDrive2->SetInverted(true);
		lDrive2->SetInverted(true);

		rIntake->Set(ControlMode::Follower, 3);
		rIntake->SetInverted(true);

		lDrive2->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		rDrive1->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);
		arm->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeoutMs);


	}
	void DisabledPeriodic() {

	}
	void AutonomousInit() {
		p = SmartDashboard::GetNumber("Arm p", p);
		i = SmartDashboard::GetNumber("Arm i", i);
		d = SmartDashboard::GetNumber("Arm d", d);
		armRate = SmartDashboard::GetNumber("Arm rate", armRate);

		arm->Config_kP(0, p, kTimeoutMs);
		arm->Config_kI(0, i, kTimeoutMs);
		arm->Config_kD(0, d, kTimeoutMs);
	}
	void AutonomousPeriodic() {

	}
	void TeleopInit() {
		armPos = 0;

		arm->SetSelectedSensorPosition(0, 0, kTimeoutMs);

		p = SmartDashboard::GetNumber("Arm p", p);
		i = SmartDashboard::GetNumber("Arm i", i);
		d = SmartDashboard::GetNumber("Arm d", d);
		armRate = SmartDashboard::GetNumber("Arm rate", armRate);

		arm->Config_kP(0, p, kTimeoutMs);
		arm->Config_kI(0, i, kTimeoutMs);
		arm->Config_kD(0, d, kTimeoutMs);
	}
	void TeleopPeriodic() {

		/*if(dr->GetRawButton(2)){
		 reverse = -1;
		 }
		 else if(dr->GetRawButton(1)){
		 reverse = 1;
		 }*/

		//lDrive1->Set(ControlMode::PercentOutput, .8 * dr->GetRawAxis(1) * reverse + .6 * dr->GetRawAxis(4));
		//rDrive1->Set(ControlMode::PercentOutput, .8 * dr->GetRawAxis(1) * reverse - .6 * dr->GetRawAxis(4));
		arm->Set(ControlMode::Position, armPos);
		lIntake->Set(ControlMode::PercentOutput,
				op->GetRawAxis(2) - op->GetRawAxis(3));
		if (op->GetRawButton(6)) {
			intakes->Set(DoubleSolenoid::kForward);
		} else if (op->GetRawButton(5)) {
			intakes->Set(DoubleSolenoid::kReverse);
		}
		if (dr->GetRawButton(1)) {
			lDrive2->SetSelectedSensorPosition(0, 0, kTimeoutMs);
			rDrive1->SetSelectedSensorPosition(0, 0, kTimeoutMs);
		}
		if (abs(op->GetRawAxis(1)) > .1) {
			armPos += op->GetRawAxis(1) * armRate;
		}
		SmartDashboard::PutNumber("Current", PDP->GetTotalCurrent());
		SmartDashboard::PutBoolean("Limit switch active", limitSwitch->Get());
		SmartDashboard::PutNumber("l encoder",
				lDrive2->GetSelectedSensorPosition());
		SmartDashboard::PutNumber("r encoder",
				rDrive1->GetSelectedSensorPosition());

		SmartDashboard::PutNumber("Arm position", armPos);
	}

	void DisabledInit() {

	}

};

START_ROBOT_CLASS(Robot)
