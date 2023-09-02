// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootReverse;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IDConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	 public static DriveConstants driveConstants = new DriveConstants();
	 public static ShooterConstants shooterConstants = new ShooterConstants();
	 public static IntakeConstants intakeConstants = new IntakeConstants();
	 public static IDConstants idConstants = new IDConstants();



	public static CommandXboxController driverController = new CommandXboxController(1);
	public static CommandXboxController operatorController = new CommandXboxController(0);

	public static IntakeSubsystem intakeSubsystem;

	public static ShooterSubsystem shooterSubsystem;

	public static DriveSubsystem driveSubsystem;

	public static PneumaticsSubsystem pneumaticsSubsystem;

	public SendableChooser<Command> chooser;

	public RobotContainer() {

		intakeSubsystem = new IntakeSubsystem();
		shooterSubsystem = new ShooterSubsystem();
		pneumaticsSubsystem = new PneumaticsSubsystem();
		driveSubsystem = new DriveSubsystem();


		configureButtonBindings();
		putAuton();

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		
		operatorController.leftBumper().whileTrue(new ShootReverse(shooterSubsystem));
		operatorController.povDown()
				.whileTrue(new IntakeCommand(1, 0, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem));
		operatorController.a()
				.whileTrue(new ShooterCommand(shooterSubsystem, intakeSubsystem, 2000, 2000, 2000)); // Static
	}

	public void putAuton() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return null;
	}
}
