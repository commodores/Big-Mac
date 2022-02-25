// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCommand extends CommandBase {
  /** Creates a new Intake. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.m_intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.m_intake.getLimitSwitch()){
      RobotContainer.m_intake.runIntake1(-1);
      RobotContainer.m_intake.runIntake2(-1);
      RobotContainer.m_intake.runIntake3(-.5);
    } else {
      RobotContainer.m_intake.runIntake1(-1);
      RobotContainer.m_intake.runIntake2(-1);
      RobotContainer.m_intake.stopIntake3();
    }
    

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
