// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootReverse extends CommandBase {
  /** Creates a new ShootReverse. */
  ShooterSubsystem shooterSubsystem;
  public ShootReverse(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterSubsystem.setKMPercentOutput(-.25);
    this.shooterSubsystem.setSM1PercentOutput(-.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
