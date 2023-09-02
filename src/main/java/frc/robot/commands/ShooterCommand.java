// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
	/** Creates a new ShooterCommand. */
	ShooterSubsystem shooterSubsystem;
	IntakeSubsystem intakeSubsystem;

	// Class variables for SM1Speed and KMSpeed
	double SM1Speed;
	double SM2Speed;
	double KMSpeed;
	double intakeSpeed = 0.2;

	public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
			int SM1Speed, int SM2Speed, int KMSpeed) {
		this.shooterSubsystem = shooterSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.SM1Speed = SM1Speed;
		this.SM2Speed = SM2Speed;
		this.KMSpeed = KMSpeed;

	}

	/***
	 * Version of the ShooterCommand constructor that manually sets the intake
	 * speed.
	 */
	public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
			int SM1Speed, int SM2Speed, int KMSpeed, double intakeSpeed) {
		this.shooterSubsystem = shooterSubsystem;
		this.intakeSubsystem = intakeSubsystem;
		this.SM1Speed = SM1Speed;
		this.SM2Speed = SM2Speed;
		this.KMSpeed = KMSpeed;
		this.intakeSpeed = intakeSpeed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (shooterSubsystem.mainWheelsAtTarget(500) && shooterSubsystem.backWheelsAtTarget(500))
			shooterSubsystem.setKickerWheelsSpeed(KMSpeed);
		intakeSubsystem.setIntakeSpeed(intakeSpeed);

		shooterSubsystem.setMainWheelsSpeed(SM1Speed);
		shooterSubsystem.setBackWheelsSpeed(SM2Speed);

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooterSubsystem.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}