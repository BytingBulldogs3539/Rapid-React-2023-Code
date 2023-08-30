// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utilities.GearRatio;

public class IntakeSubsystem extends SubsystemBase {

	TalonFX intakeMotor;
	TalonSRX knockDownMotor;

	DigitalInput sensor;

	final boolean hasIntakeMotor;
	final boolean hasKnockDownMotor;

	public IntakeSubsystem() {
		if (RobotContainer.constants.getIntakeConstants().getIntakeMotorID() != -1) {
			hasIntakeMotor = true;
			intakeMotor = new TalonFX(RobotContainer.constants.getIntakeConstants().getIntakeMotorID());
			GearRatio gearRatio = RobotContainer.constants.getIntakeConstants().getIntakeGearRatio();
			intakeMotor.setInverted(gearRatio.getInverted());
			intakeMotor.setSensorPhase(gearRatio.getInverted());
			intakeMotor.configSupplyCurrentLimit(
					new SupplyCurrentLimitConfiguration(true, gearRatio.getCurrentLimit(), gearRatio.getCurrentLimit(),
							0));
		} else {
			hasIntakeMotor = false;
		}

		if (RobotContainer.constants.getIntakeConstants().getKnockDownMotorID() != -1) {
			hasKnockDownMotor = true;
			knockDownMotor = new TalonSRX(RobotContainer.constants.getIntakeConstants().getKnockDownMotorID());
			GearRatio gearRatio = RobotContainer.constants.getIntakeConstants().getKnockDownGearRatio();
			knockDownMotor.setInverted(gearRatio.getInverted());
			knockDownMotor.setSensorPhase(gearRatio.getInverted());
			knockDownMotor.configSupplyCurrentLimit(
					new SupplyCurrentLimitConfiguration(true, gearRatio.getCurrentLimit(), gearRatio.getCurrentLimit(),
							0));
		} else {
			hasKnockDownMotor = false;
		}

	}

	/**
	 * @param speed a value between -1 & 1
	 */
	public void setIntakeSpeed(double speed) {
		if (!hasIntakeMotor) {
			return;
		}
		intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
	}

	/**
	 * 
	 * @param speed a value between -1 & 1
	 */
	public void setKnockDownSpeed(double speed) {
		if (!hasKnockDownMotor) {
			return;
		}
		knockDownMotor.set(ControlMode.PercentOutput, speed * RobotContainer.constants.getIntakeConstants().getKnockDownGearRatio().getGearRatio());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
