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

	Timer PIDTimer;

	double pi = 3.1415;
	double reverse = -1;
	double armPos = 0;

	double armSensor = 0;
	double iZone;
	double pTerm;
	double iTerm;
	double dTerm;
	double error;
	double prevTime;
	double prevPos;
	double maxSensorPos = 4461;
	double degreesPerTick = 180 / maxSensorPos;

	// We get arm rate from driver station
	double armRate = .1;

	// PID for the arm
	double p = .001, i = .001, d = 1;

	TalonSRX * lDrive1 = new TalonSRX(8);
	TalonSRX * lDrive2 = new TalonSRX(11);
	TalonSRX * rDrive1 = new TalonSRX(7);
	TalonSRX * rDrive2 = new TalonSRX(14);
	TalonSRX * lIntake = new TalonSRX(3);
	TalonSRX * rIntake = new TalonSRX(2);

	// Setup the arm
	TalonSRX * arm = new TalonSRX(9);

	DoubleSolenoid * intakes = new DoubleSolenoid(0, 1);
	Joystick * dr = new Joystick(0);
	Joystick * op = new Joystick(1);

	// Add an object for the limit switch
	int limitSwitchPort = 9; // Limit switch port is number 9 (by the RSL)
	DigitalInput * limitSwitch = new DigitalInput(limitSwitchPort);

	PowerDistributionPanel * PDP = new PowerDistributionPanel();

	void RobotInit() {

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

		// Setup the encoder for the arm
		arm->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
		arm->SetSensorPhase(true);

	}
	void DisabledPeriodic() {

	}
	void AutonomousInit() {

	}
	void AutonomousPeriodic() {

	}
	void TeleopInit() {

		maxSensorPos = 4461;

		p = SmartDashboard::GetNumber("Arm p", p);
		i = SmartDashboard::GetNumber("Arm i", i);
		d = SmartDashboard::GetNumber("Arm d", d);
		armRate = SmartDashboard::GetNumber("Arm rate", armRate);

		arm->Config_kP(0, p, kTimeoutMs);
		arm->Config_kI(0, i, kTimeoutMs);
		arm->Config_kD(0, d, kTimeoutMs);
		arm->Config_kF(0, 0, kTimeoutMs);

		armPos = 0;

		arm->SetSelectedSensorPosition(0, 0, kTimeoutMs);

		p = SmartDashboard::GetNumber("Arm p", p);
		i = SmartDashboard::GetNumber("Arm i", i);
		d = SmartDashboard::GetNumber("Arm d", d);
		armRate = SmartDashboard::GetNumber("Arm rate", armRate);

		arm->Config_kP(0, p, kTimeoutMs);
		arm->Config_kI(0, i, kTimeoutMs);
		arm->Config_kD(0, d, kTimeoutMs);

		PIDTimer.Start();
	}
	void TeleopPeriodic() {

		armSensor = arm->GetSelectedSensorPosition(0);

		//error = armPos - armSensor;

		//pTerm = error * p;

		/*if (abs(error) < iZone) {
		 iTerm += ((PIDTimer.Get() - prevTime)
		 * (armSensor - prevPos)) * i;
		 }
		 else{
		 iTerm = 0;
		 }

		 dTerm = ((armSensor - prevPos)
		 / (PIDTimer.Get() - prevTime)) * d;

		 */

		if(dr->GetRawButton(2)){
		 reverse = -1;
		 }
		 else if(dr->GetRawButton(1)){
		 reverse = 1;
		 }
		/*if (op->GetRawButton(2)) {
			if (armSensor < 90 / degreesPerTick;) {
				armPos = 135/degreesPerTick;
			} else {
				armPos = 45/degreesPerTick;
			}
		}*/
		arm->Set(ControlMode::Position, armPos);
		lDrive1->Set(ControlMode::PercentOutput, .8 * dr->GetRawAxis(1) * reverse + .6 * dr->GetRawAxis(4));
		rDrive1->Set(ControlMode::PercentOutput, .8 * dr->GetRawAxis(1) * reverse - .6 * dr->GetRawAxis(4));
		//arm->Set(ControlMode::PercentOutput, (dTerm + iTerm + pTerm) * .5 + cos(armSensor * degreePerTick) * .5);
		//arm->Set(ControlMode::PercentOutput, op->GetRawAxis(1));
		SmartDashboard::PutNumber("angle on SRX", armSensor * degreesPerTick);
		//SmartDashboard::PutNumber("arm out", cos(armSensor * degreesPerTick) * .3);
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

		SmartDashboard::PutNumber("Arm target position",
				armPos * degreesPerTick);
		SmartDashboard::PutNumber("Arm sensor value", armSensor);
		SmartDashboard::PutNumber("Arm output on SRX",
				arm->GetMotorOutputPercent());
		SmartDashboard::PutNumber("Arm target on SRX",
				arm->GetClosedLoopTarget(0));
		SmartDashboard::PutNumber("Arm error on SRX",
				arm->GetClosedLoopError(0));

		//prevTime = PIDTimer.Get();
		//prevPos = armSensor;
	}

	void DisabledInit() {

	}

};

START_ROBOT_CLASS(Robot)
