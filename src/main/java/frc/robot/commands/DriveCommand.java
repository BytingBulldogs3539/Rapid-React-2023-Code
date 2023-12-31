package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
	private final DriveSubsystem drivetrain;

	public DriveCommand(DriveSubsystem drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
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
		value = deadband(value, 0.1);

		// Square the axis
		value = Math.copySign(value * value, value);

		return value;
	}

	@Override
	public void execute() {
		double translationXPercent = modifyAxis(RobotContainer.driverController.getLeftY());
		double translationYPercent = -modifyAxis(RobotContainer.driverController.getLeftX());
		double rotationPercent = -modifyAxis(RobotContainer.driverController.getRightX());

		drivetrain.drive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
						-DriveConstants.driveRatio * translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
						DriveConstants.driveRatio * translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
						DriveConstants.steerRatio * rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
						Rotation2d.fromDegrees(0)));
 }

	@Override
	public void end(boolean interrupted) {
		// Stop the drivetrain
		drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}