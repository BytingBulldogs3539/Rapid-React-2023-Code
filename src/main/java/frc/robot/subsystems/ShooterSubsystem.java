// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDConstants;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
	// Declares shooter motor objects, but does not initialize them yet.
	TalonFX mainWheels;
	TalonFX backWheels;
	TalonFX kickerWheels;

	final Double VELCONV = (2048.0 / 600.0);

	// Creates a new object for the ball sensor (light beam sensor).
	DigitalInput sensor;

	// Create color sensor and corosponding color matcher.
	// ColorSensorV3 colorSensor;
	// private final ColorMatch colorMatcher = new ColorMatch();

	public ShooterSubsystem() {
		int mainWheelsID = IDConstants.mainWheelsID;
		mainWheels = configureMotor(mainWheelsID, ShooterConstants.mainWheelsInvert,
				ShooterConstants.mainWheelsCurrentLimit, ShooterConstants.mainWheelsKP, ShooterConstants.mainWheelsKI,
				ShooterConstants.mainWheelsKD, ShooterConstants.mainWheelsKF);
		mainWheels.setNeutralMode(NeutralMode.Coast);

		int backWheelsID = IDConstants.backWheelsID;

		backWheels = configureMotor(backWheelsID, ShooterConstants.backWheelsInvert,
		ShooterConstants.backWheelsCurrentLimit, ShooterConstants.backWheelsKP, ShooterConstants.backWheelsKI,
		ShooterConstants.backWheelsKD, ShooterConstants.backWheelsKF);
		backWheels.setNeutralMode(NeutralMode.Coast);

		int kickerWheelsID = IDConstants.kickerWheelsID;

		kickerWheels = configureMotor(kickerWheelsID, ShooterConstants.kickerWheelsInvert,
		ShooterConstants.kickerWheelsCurrentLimit, ShooterConstants.kickerWheelsKP, ShooterConstants.kickerWheelsKI,
		ShooterConstants.kickerWheelsKD, ShooterConstants.kickerWheelsKF);
		kickerWheels.setNeutralMode(NeutralMode.Brake);

		sensor = new DigitalInput(IDConstants.digitalInputID);
	}

	public TalonFX configureMotor(int motorID, boolean inverted, int currentLimit, double kp, double ki, double kd,
			double kf) {
		TalonFX motor = new TalonFX(motorID);
		motor.setInverted(inverted);
		motor.setSensorPhase(inverted);
		motor.configSupplyCurrentLimit(
				new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 0));
		setPIDConstants(motor, kp, ki, kd, kf);

		return motor;
	}

	/***
	 * Configures PID constants for the given shooter motor
	 * 
	 * @param motor        (motor whose PID constants will be set)
	 * @param PIDConstants (PID constants to set the shooter motor's PID constants
	 *                     to)
	 */
	public void setPIDConstants(TalonFX motor, double kp, double ki, double kd, double kf) {
		if (motor != null) {
			motor.config_kP(0, kp);
			motor.config_kI(0, ki);
			motor.config_kD(0, kd);
			motor.config_kF(0, kf);
			motor.configMaxIntegralAccumulator(0, 0);
		}
	}

	/**
	 * @return returns true when the sensor in the shooter is blocked. Also returns
	 *         true if the robot does not have a sensor.
	 */
	public boolean getSensor() {
		if (ShooterConstants.invertSensor) {
			return !sensor.get();
		}
		return sensor.get();
	}

	/***
	 * Sets the percent output of all of the shooter motors
	 * 
	 * @param mainWheelsSpeed (Percent output to set the first shooter motor to)
	 * @param backWheelsSpeed (Percent output to set the second shooter motor to)
	 * @param SM3Speed (Percent output to set the third shooter motor to)
	 * @param KMSpeed  (Percent output to set the kicker motor to)
	 */
	public void setShooterPercentOutput(double mainWheelsSpeed, double backWheelsSpeed, double KMSpeed) {
		mainWheels.set(TalonFXControlMode.PercentOutput,
				mainWheelsSpeed * ShooterConstants.mainWheelsGearRatio);
		backWheels.set(TalonFXControlMode.PercentOutput, backWheelsSpeed);
		kickerWheels.set(TalonFXControlMode.PercentOutput,
				KMSpeed * ShooterConstants.kickerWheelsGearRatio);
	}

	/***
	 * Sets the percent output of the first shooter motor.
	 * 
	 * @param mainWheelsSpeed (Percent output to set the first shooter motor to)
	 */
	public void setMainWheelsPercentOutput(double mainWheelsSpeed) {
		mainWheels.set(TalonFXControlMode.PercentOutput,
				mainWheelsSpeed * ShooterConstants.mainWheelsGearRatio);
	}

	/***
	 * Sets the percent output of the second shooter motor.
	 * 
	 * @param backWheelsSpeed (Percent output to set the second shooter motor to)
	 */
	public void setBackWheelsPercentOutput(double backWheelsSpeed) {
		backWheels.set(TalonFXControlMode.PercentOutput, backWheelsSpeed);
	}

	/***
	 * Sets the percent output of the first shooter motor.
	 * 
	 * @param KMSpeed (Percent output to set the kicker motor to)
	 */
	public void setKickerWheelsPercentOutput(double KMSpeed) {
		kickerWheels.set(TalonFXControlMode.PercentOutput,
				KMSpeed * ShooterConstants.kickerWheelsGearRatio);
	}

	/**
	 * Sets the speed of every shooter motor.
	 * 
	 * @param speed (Value to set the speed of every shooter motor to)
	 */
	public void setSMSpeeds(double speed) {
		mainWheels.set(TalonFXControlMode.Velocity,
				speed * VELCONV * ShooterConstants.mainWheelsGearRatio);
		backWheels.set(TalonFXControlMode.Velocity,
				speed * VELCONV * ShooterConstants.backWheelsGearRatio);
		kickerWheels.set(TalonFXControlMode.Velocity,
				speed * VELCONV * ShooterConstants.kickerWheelsGearRatio);
	}

	/**
	 * Sets the speed of the first shooter motor.
	 * 
	 * @param speed (Value to set the speed of the first shooter motor to in rpm)
	 */
	public void setMainWheelsSpeed(double speed) {
		mainWheels.set(TalonFXControlMode.Velocity,
				speed * VELCONV * ShooterConstants.mainWheelsGearRatio);
	}

	/**
	 * Sets the speed of the second shooter motor.
	 * 
	 * @param speed (Value to set the speed of the second shooter motor to in rpm)
	 */
	public void setBackWheelsSpeed(double speed) {
		backWheels.set(TalonFXControlMode.Velocity,
				speed * VELCONV * ShooterConstants.backWheelsGearRatio);
	}

	/**
	 * Sets the speed of the kicker motor.
	 * 
	 * @param speed (Value to set the speed of the kicker motor to in rpm)
	 */
	public void setKickerWheelsSpeed(double speed) {
		kickerWheels.set(TalonFXControlMode.Velocity,
				speed * VELCONV * ShooterConstants.kickerWheelsGearRatio);
	}

	/**
	 * 
	 * @param tolerance in rpm
	 * @return is or is not within the tolerance of the set rpm.
	 */
	public boolean mainWheelsAtTarget(double tolerance) {
		return (mainWheels.getClosedLoopError() / VELCONV) < tolerance;
	}

	/**
	 * 
	 * @param tolerance in rpm
	 * @return is or is not within the tolerance of the set rpm.
	 */
	public boolean backWheelsAtTarget(double tolerance) {
		return (backWheels.getClosedLoopError() / VELCONV) < tolerance;
	}

	/**
	 * 
	 * @param tolerance in rpm
	 * @return is or is not within the tolerance of the set rpm.
	 */
	public boolean kickerWheelsAtTarget(double tolerance) {
		return (kickerWheels.getClosedLoopError() / VELCONV) < tolerance;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run.
	}

	public void stop() {
		mainWheels.set(TalonFXControlMode.PercentOutput, 0);

		backWheels.set(TalonFXControlMode.PercentOutput, 0);

		kickerWheels.set(TalonFXControlMode.PercentOutput, 0);
	}
}