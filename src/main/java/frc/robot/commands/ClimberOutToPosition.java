// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimberOutToPosition extends CommandBase {
  private double distance;
  /** Creates a new ClimberUpToPosition. */
  public ClimberOutToPosition(double getDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = getDistance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_climberRotate.climberRotate(.85);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_climberRotate.stopClimberRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_climberRotate.getRotateEncoder() >= distance;
  }
}
