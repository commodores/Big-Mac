// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberNew;

public class ClimberRotateManual extends CommandBase {
  /** Creates a new ClimberRotateManual. */
  private final ClimberRotate m_climberRotate;
  public ClimberRotateManual(ClimberRotate ClimberRotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climberRotate = ClimberRotate;
    addRequirements(ClimberRotate);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_arcade1.getRawAxis(1) > 0.5 ){
      m_climberRotate.climberRotate(-.85);
    } else if(RobotContainer.m_arcade1.getRawAxis(1) < -0.5 ) {
      m_climberRotate.climberRotate(.85);
    }else {
      m_climberRotate.stopClimberRotate();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
