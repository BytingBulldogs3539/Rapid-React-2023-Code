// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SdsSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.ShooterSubsystem.Color;

public class DriveSubsystem extends SubsystemBase {
	public static final double MAX_VOLTAGE = 12.0;

	public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
			DriveConstants.wheelTrackLength *
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

	SwerveDriveOdometry m_odometry;

	private final Pigeon2 m_pigeon = new Pigeon2(RobotContainer.constants.getDriveConstants().getPigeonID());

	Pose2d m_pose;

	private final SwerveModule m_frontLeftModule;
	private final SwerveModule m_frontRightModule;
	private final SwerveModule m_backLeftModule;
	private final SwerveModule m_backRightModule;

	private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	public DriveSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		m_frontLeftModule = SdsSwerveModuleHelper.createFalcon500(
				// This parameter is optional, but will allow you to see the current state of
				// the module on the dashboard.
				tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,
						0),
				// This can either be STANDARD or FAST depending on your gear configuration
				getModuleConfig(),
				// This is the ID of the drive motor
				RobotContainer.constants.getDriveConstants().getFLDriveID(),
				// This is the ID of the steer motor
				RobotContainer.constants.getDriveConstants().getFLSteeringID(),
				// This is the ID of the steer encoder
				RobotContainer.constants.getDriveConstants().getFLCanEncoderID(),
				// This is how much the steer encoder is offset from true zero (In our case,
				// zero is facing straight forward)
				RobotContainer.constants.getDriveConstants().getFLSteerOffset());

		// We will do the same for the other modules
		m_frontRightModule = SdsSwerveModuleHelper.createFalcon500(
				tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,
						0),
				getModuleConfig(),
				RobotContainer.constants.getDriveConstants().getFRDriveID(),
				RobotContainer.constants.getDriveConstants().getFRSteeringID(),
				RobotContainer.constants.getDriveConstants().getFRCanEncoderID(),
				RobotContainer.constants.getDriveConstants().getFRSteerOffset());

		m_backLeftModule = SdsSwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,
						0),
				getModuleConfig(),
				RobotContainer.constants.getDriveConstants().getBLDriveID(),
				RobotContainer.constants.getDriveConstants().getBLSteeringID(),
				RobotContainer.constants.getDriveConstants().getBLCanEncoderID(),
				RobotContainer.constants.getDriveConstants().getBLSteerOffset());

		m_backRightModule = SdsSwerveModuleHelper.createFalcon500(
				tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,
						0),
				getModuleConfig(),
				RobotContainer.constants.getDriveConstants().getBRDriveID(),
				RobotContainer.constants.getDriveConstants().getBRSteeringID(),
				RobotContainer.constants.getDriveConstants().getBRCanEncoderID(),
				RobotContainer.constants.getDriveConstants().getBRSteerOffset());

		zeroGyroscope();

		m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(),
				null, new Pose2d(0, 0, new Rotation2d()));

		setDefaultCommand(new DriveCommand(this));
	}

	/**
	 * Used to zero the pigeon / gyroscope.
	 */
	public void zeroGyroscope() {
		m_pigeon.setYaw(0);
	}

	/**
	 * Used to get the angle of rotation of the robot from the last time the gyro
	 * was zeroed.
	 * 
	 * @returns gyro angle.
	 */
	public Rotation2d getGyroscopeRotation() {
		return Rotation2d.fromDegrees(m_pigeon.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		m_chassisSpeeds = chassisSpeeds;
	}

	public ModuleConfiguration getModuleConfig() {
		return new ModuleConfiguration(DriveConstants.wheelDiameter, DriveConstants.driveGearReduction, false,
				DriveConstants.steerGearReduction,
				false);
	}

	public SwerveModuleState getState(SwerveModule swerveModule) {
		return new SwerveModuleState(swerveModule.getPosition().distanceMeters, swerveModule.getPosition().angle);
	}

	public Pose2d getPose() {
		return m_pose;
	}

	public void resetPose(Pose2d pose) {
		m_odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
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

		m_pose = m_odometry.update(getGyroscopeRotation(), getModulePositions());
	}
}