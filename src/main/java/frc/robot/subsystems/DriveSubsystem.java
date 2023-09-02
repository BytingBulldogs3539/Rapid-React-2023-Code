// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IDConstants;

public class DriveSubsystem extends SubsystemBase {
	public static final double MAX_VOLTAGE = 12.0;

	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			DriveConstants.wheelDiameter * Math.PI;

	public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
			Math.hypot(DriveConstants.wheelTrackWidth / 2.0,
					DriveConstants.wheelTrackLength / 2.0);

	private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DriveConstants.wheelTrackWidth / 2.0,
					DriveConstants.wheelTrackLength / 2.0),
			// Front right
			new Translation2d(DriveConstants.wheelTrackWidth / 2.0,
					-DriveConstants.wheelTrackLength / 2.0),
			// Back left
			new Translation2d(-DriveConstants.wheelTrackWidth / 2.0,
					DriveConstants.wheelTrackLength / 2.0),
			// Back right
			new Translation2d(-DriveConstants.wheelTrackWidth / 2.0,
					-DriveConstants.wheelTrackLength / 2.0));

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public DriveSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		ModuleConfiguration moduleGearRatio = new ModuleConfiguration(DriveConstants.wheelDiameter,
				DriveConstants.driveGearReduction, false,
				DriveConstants.steerGearReduction,
				false);

		m_frontLeftModule = SdsSwerveModuleHelper.createFalcon500(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
						0),

				// Gear Ratio of the module
				moduleGearRatio,
				// This is the ID of the drive motor
				IDConstants.FLDriveID,

				"roborio",
				// This is the ID of the steer motor
				IDConstants.FLSteeringID,

				"roborio",
				// This is the ID of the steer encoder
				IDConstants.FLCanEncoderID,
				"roborio",
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				DriveConstants.FLSteerOffset);

		// We will do the same for the other modules
		m_frontRightModule = SdsSwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,
						0),
				moduleGearRatio,
				IDConstants.FRDriveID,
				"roborio",
				IDConstants.FRSteeringID,
				"roborio",
				IDConstants.FRCanEncoderID,
				"roborio",
				DriveConstants.FRSteerOffset);

		m_backLeftModule = SdsSwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,
						0),
				moduleGearRatio,
				IDConstants.BLDriveID,
				"roborio",
				IDConstants.BLSteeringID,
				"roborio",
				IDConstants.BLCanEncoderID,
				"roborio",
				DriveConstants.BLSteerOffset);

		m_backRightModule = SdsSwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,
						0),
				moduleGearRatio,
				IDConstants.BRDriveID,
				"roborio",
				IDConstants.BRSteeringID,
				"roborio",
				IDConstants.BRCanEncoderID,
				"roborio",
				DriveConstants.BRSteerOffset);

		setDefaultCommand(new DriveCommand(this));
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public SwerveModuleState getState(SwerveModule swerveModule) {
		return new SwerveModuleState(swerveModule.getPosition().distanceMeters, swerveModule.getPosition().angle);
	}

	public ChassisSpeeds getChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState(m_frontLeftModule),
				getState(m_frontRightModule),
				getState(m_backLeftModule), getState(m_backRightModule));
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = {
				m_frontLeftModule.getPosition(),
				m_frontRightModule.getPosition(),
				m_backLeftModule.getPosition(),
				m_backRightModule.getPosition()
		};
		return positions;
	}

	@Override
	public void periodic() {
		SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians());
		m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());

	}
}