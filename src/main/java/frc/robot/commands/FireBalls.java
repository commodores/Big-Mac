// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class FireBalls extends CommandBase {
  /** Creates a new FireBalls. */
  public FireBalls() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    RobotContainer.m_intake.runIntake1(-1);
    RobotContainer.m_intake.runIntake2(-1);
    RobotContainer.m_intake.runIntake3(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intake.stopIntake1();
    RobotContainer.m_intake.stopIntake2();
    RobotContainer.m_intake.stopIntake3();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
