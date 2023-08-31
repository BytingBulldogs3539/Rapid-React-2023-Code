// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PneumaticsSubsystem extends SubsystemBase {
	// Final variables for pressure on and off
	final double minPressure = 80;
	final double maxPressure = 120;

	// Creates compressor & solenoid objects
	Compressor compressor;
	DoubleSolenoid intakeSolenoid;
	DoubleSolenoid climberSolenoid;

	/** Creates a new Pneumatics. */
	public PneumaticsSubsystem() {
		// Creates a compressor object
		compressor = new Compressor(RobotContainer.constants.getPneumaticsConstants().getCompressorID(),
				PneumaticsModuleType.REVPH);
		compressor.enableAnalog(minPressure, maxPressure);
		compressor.disable();

		// Creates 1 solenoid objects
		intakeSolenoid = new DoubleSolenoid(RobotContainer.constants.getPneumaticsConstants().getCompressorID(),
				PneumaticsModuleType.REVPH, RobotContainer.constants.getPneumaticsConstants().getIntakeSolenoidIn(),
				RobotContainer.constants.getPneumaticsConstants().getIntakeSolenoidOut());


		// Add the compressor to the dashboard to see when its running.
		SmartDashboard.putData(compressor);
		setIntakeIn();
	}

	public void setIntakeIn() {
		intakeSolenoid.set(Value.kForward);
	}

	public void setIntakeOut() {
		intakeSolenoid.set(Value.kReverse);
	}

	public void enableCompressor() {
		compressor.enableAnalog(minPressure, maxPressure);
	}

	public void disableCompressor() {
		compressor.disable();
	}

	@Override
	public void periodic() {
	}
}
