package frc.robot.constants;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.control.MaxAccelerationConstraint;
import com.swervedrivespecialties.swervelib.control.MaxVelocityConstraint;
import com.swervedrivespecialties.swervelib.control.TrajectoryConstraint;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utilities.GearRatio;
import frc.robot.utilities.PIDConstants;

public class CompConstants{

	
	public DriveConstants getDriveConstants() {
		return new DriveConstants();
	}

	
	public IntakeConstants getIntakeConstants() {
		return new IntakeConstants();
	}

	
	public ShooterConstants getShooterConstants() {
		return new ShooterConstants();
	}

	
	public ClimberConstants getClimberConstants() {
		return new ClimberConstants();
	}

	
	public PneumaticsConstants getPneumaticsConstants() {
		return new PneumaticsConstants();
	}

	
	public LightsConstants getLightsConstants() {
		return new LightsConstants();
	}

	public class DriveConstants {
		
		public double getWheelDiameter() {
			// TODO Auto-generated method stub
			return 0.0983;
		}

		
		public double getWheelTrackWidth() {
			// TODO Auto-generated method stub
			return 0.3302;
		}

		
		public double getWheelBase() {
			// TODO Auto-generated method stub
			return 0.3302;
		}

		
		public int getFLDriveID() {
			return 2;
		}

		
		public int getFLSteeringID() {
			return 1;
		}

		
		public int getFRDriveID() {
			return 18;
		}

		
		public int getFRSteeringID() {
			return 17;
		}

		
		public int getBLDriveID() {
			return 4;
		}

		
		public int getBLSteeringID() {
			return 3;
		}

		
		public int getBRDriveID() {
			return 42; // Was 14
		}

		
		public int getBRSteeringID() {
			return 41; // Was 13
		}

		
		public int getFLCanEncoderID() {
			return 30;
		}

		
		public int getFRCanEncoderID() {
			return 33;
		}

		
		public int getBLCanEncoderID() {
			return 31;
		}

		
		public int getBRCanEncoderID() {
			return 32;
		}

		
		public int getPigeonID() {
			// TODO Auto-generated method stub
			return 50;
		}

		
		public double getFLSteerOffset() {
			// TODO Auto-generated method stub
			return -5.3274;
		}

		
		public double getFRSteerOffset() {
			// TODO Auto-generated method stub
			return -1.3237;
		}

		
		public double getBLSteerOffset() {
			return -3.9284;
		}

		
		public double getBRSteerOffset() {
			return -2.932851;
		}

		
		public double getDriveGearReduction() {
			return 1.0 / 7.13;
		}

		
		public double getSteerGearReduction() {
			// Eight to twenty-four, fourteen to seventy-two			
			return -1*(8.0 / 24.0 * 14.0 / 72.0);
		}

		
		public ModuleConfiguration getFLModuleGearRatio() {
			return new ModuleConfiguration(getWheelDiameter(), getDriveGearReduction(), false, getSteerGearReduction(),
					false); // Add appropriate parameter values when you have them.
		}

		
		public ModuleConfiguration getFRModuleGearRatio() {
			return new ModuleConfiguration(getWheelDiameter(), getDriveGearReduction(), false, getSteerGearReduction(),
					false); // Add appropriate parameter values when you have them.
		}

		
		public ModuleConfiguration getBLModuleGearRatio() {
			return new ModuleConfiguration(getWheelDiameter(), getDriveGearReduction(), false, getSteerGearReduction(),
					false); // Add appropriate parameter values when you have them.
		}

		
		public ModuleConfiguration getBRModuleGearRatio() {
			return new ModuleConfiguration(getWheelDiameter(), getDriveGearReduction(), false, getSteerGearReduction(),
					false); // Add appropriate parameter values when you have them.
		}

		
		public String getFrontCameraName() {
			// TODO Auto-generated method stub
			return "HD_USB_Camera";
		}

		
		public String getShooterCameraName() {
			// TODO Auto-generated method stub
			return "mmal_service_16.1";
		}

		
		public double getFrontCameraHeightMeters() {
			// TODO Auto-generated method stub
			return 0;
		}

		
		public double getFrontCameraPitchRadians() {
			// TODO Auto-generated method stub
			return 0;
		}

		
		public double getShooterCameraHeightMeters() {
			return Units.inchesToMeters(29);
		}

		
		public double getShooterCameraPitchRadians() {
			return Units.degreesToRadians(42);
		}

		
		public PIDConstants getFrontCameraPIDConstants() {
			return new PIDConstants(0.03, .001, 0);
		}

		
		public PIDConstants getShooterCameraPIDConstants() {
			return new PIDConstants(0.03, 0.001, 0.0);
		}

		
		public PIDConstants getTranslationXPIDConstants() {
			return new PIDConstants(20, 3, 0, 1.7);
		}

		
		public PIDConstants getTranslationYPIDConstants() {
			return new PIDConstants(20, 3, 0, 1.7);
		}

		
		public PIDConstants getRotationConstants() {
			return new PIDConstants(1, 0, .07, 0);
		}

		
		public TrajectoryConstraint[] getConstraints() {
			TrajectoryConstraint[] constraints = { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
					(TrajectoryConstraint) new MaxVelocityConstraint(2) };
			return constraints;
		}

		
		public int getPDHID() {
			// TODO Auto-generated method stub
			return -1;
		}
	}

	public class IntakeConstants{

		
		public int getIntakeMotorID() {
			return 0;
		}

		
		public int getKnockDownMotorID() {
			return 19;
		}

		
		public GearRatio getIntakeGearRatio() {
			return new GearRatio(1, true, 40); // Gear ratio does not matter but direction does.
		}

		
		public GearRatio getKnockDownGearRatio() {
			return new GearRatio(0.5, true, 40); // Gear ratio impacts the speed that the knockdown belts move at.
		}

	}

	public class ShooterConstants {

		
		public int getSM1ID() {
			return 11;
		}

		
		public int getSM2ID() {
			return 5;
		}

		
		public PIDConstants getPIDConstants() {
			return new PIDConstants(0.1, 0, 5, 0.045);
		}

		
		public PIDConstants getTopPIDConstants() {
			return new PIDConstants(0.005, 0, 0, 0.045);
		}

		
		public GearRatio getSM1GearRatio() {
			return new GearRatio(1, true, 40);
		}

		
		public int getSM3ID() {
			return -1;
		}

		
		public int getKMID() {
			return 12;
		}

		
		public GearRatio getSM2GearRatio() {
			return new GearRatio(1, true, 40);
		}

		
		public GearRatio getSM3GearRatio() {
			return new GearRatio(1, true, 40);
		}

		
		public GearRatio getKMGearRatio() {
			return new GearRatio(2, true, 40);
		}

		
		public int getDigitalInput() {
			return 0;
		}

		
		public boolean invertSensor() {
			return true;
		}

		
		public boolean getColorSensor() {
			return true;
		}


		
		public int getBlueTolerance() {
			return 50;
		}


		
		public int getRedTolerance() {
			return 50;
		}

		
		public Color getBlueColor() {
			return new Color(0, 0, .5);
		}

		
		public Color getRedColor() {
			return new Color(.5, 0, 0);
		}

		
		public boolean getUseHood(double pitch) {
			return pitch<=-3.3;
		}
		
		// For SM1 motor speeds given target distance (located in constants)
		
		public double getShooterSpeed(double distance) {
			// return(275.889 * distance + 2206.11 + 85);
			return(180.21 * distance + 2447); // 3317.42 at 3.72 meters
			// 2930.49 at 2.68 meters
		}

		// For SM2 motor speeds given target distance (located in constants)
		
		public double getTopShooterSpeed(double distance) {
			return(703.94 * distance + 1539.06+100);
		}
	}

	public class ClimberConstants{

		
		public int getLClimberMotorID() {
			return 15;
		}

		
		public int getRClimberMotorID() {
			return 16;
		}

		
		public GearRatio getLClimberGearRatio() {
			return new GearRatio(1, false, 40); // Gear ratio does not matter but inversion does
		}

		
		public GearRatio getRClimberGearRatio() {
			return new GearRatio(1, false, 40); // Gear ratio does not matter but inversion does
		}

		
		public int getLeftLimitID() {
			// TODO Auto-generated method stub
			return 1;
		}

		
		public int getRightLimitID() {
			// TODO Auto-generated method stub
			return 2;
		}

		
		public boolean invertLeftLimitSensor() {
			// TODO Auto-generated method stub
			return true;
		}

		
		public boolean invertRightLimitSensor() {
			return true;
		}

		
		public boolean getLClimberMotorInversion() {
			return false;
		}

		
		public boolean getRClimberMotorInversion() {
			return false;
		}

		
		public int getLClimberSoftBottom() {
			// TODO Auto-generated method stub
			return -8000;
		}

		
		public int getRClimberSoftBottom() {
			// TODO Auto-generated method stub
			return -8000;
		}

		
		public int getLFrameHeight() {
			// TODO Auto-generated method stub
			return 132000- (2700*6);
		}

		
		public int getLMaxExtensionHeight() {
			// TODO Auto-generated method stub
			return 180000+ (2700*5) - (2700*7);
		}

		
		public int getRFrameHeight() {
			// TODO Auto-generated method stub
			return 132000- (2700*10);
		}

		
		public int getRMaxExtensionHeight() {
			// TODO Auto-generated method stub
			return 180000 - (2700*7);
		}
	}

	public class PneumaticsConstants{

		
		public int getCompressorID() {
			return 25;
		}

		
		public int getIntakeSolenoidIn() {
			return 3;
		}

		
		public int getIntakeSolenoidOut() {
			return 2;
		}

		
		public int getStaticClimberSolenoidOpen() {
			return 1;
		}

		
		public int getStaticClimberSolenoidClose() {
			return 0;
		}

		
		public int getClimberSolenoidIn() {
			return 5;
		}

		
		public int getClimberSolenoidOut() {
			return 4;
		}

		
		public int getShooterForward() {
			// TODO Auto-generated method stub
			return 7;
		}

		
		public int getShooterReverse() {
			// TODO Auto-generated method stub
			return 6;
		}
	}

	public class LightsConstants {
		
		public int getCandleID() {
			return 40;
		}

		
		public int getNumOfLights() {
			return 30;
		}

	}
}