// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

	TalonFX intakeMotor;

	public IntakeSubsystem() {
		intakeMotor = new TalonFX(IDConstants.intakeMotorID);
		intakeMotor.setInverted(IntakeConstants.invertDirection);
		intakeMotor.setSensorPhase(IntakeConstants.invertDirection);
		intakeMotor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, IntakeConstants.currentLimit, IntakeConstants.currentLimit,
						0));
	}

	/**
	 * @param speed a value between -1 & 1
	 */
	public void setIntakeSpeed(double speed) {
		intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
