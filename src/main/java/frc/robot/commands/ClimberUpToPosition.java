// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimberUpToPosition extends CommandBase {
  private double height;
  /** Creates a new ClimberUpToPosition. */
  public ClimberUpToPosition(double getHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    height = getHeight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_climber.climberElevate(.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_climber.stopClimberElevate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_climber.getClimberEncoder() >= height;
  }
}
