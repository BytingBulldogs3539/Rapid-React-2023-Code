package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
	private final DriveSubsystem drivetrain;

	public DriveCommand(DriveSubsystem drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);

		SmartDashboard.putNumber("Steer Ratio", 0.4);
		SmartDashboard.putNumber("Drive Ratio", 0.5);
	}

	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.05);

		// Square the axis
		value = Math.copySign(value * value, value);

		return value;
	}

	@Override
	public void execute() {
		Rotation2d gyroAngle = drivetrain.getGyroscopeRotation();
		double translationXPercent = modifyAxis(RobotContainer.driverController.getLeftStickY());
		double translationYPercent = -modifyAxis(RobotContainer.driverController.getLeftStickX());
		double rotationPercent = -modifyAxis(RobotContainer.driverController.getRightStickX());

		double driveRatio = SmartDashboard.getNumber("Drive Ratio", 0.5);
		double steerRatio = SmartDashboard.getNumber("Steer Ratio", 0.4);

		if (RobotContainer.driverController.getRightTrigger() > .1) {
			driveRatio = 1.0;
		}

		if (RobotContainer.driverController.buttonBR.getAsBoolean()) {

			gyroAngle = Rotation2d.fromDegrees(0);
		}

		drivetrain.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						driveRatio * translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
						driveRatio * translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
						steerRatio * rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
						gyroAngle));
	}

	@Override
	public void end(boolean interrupted) {
		// Stop the drivetrain
		drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}