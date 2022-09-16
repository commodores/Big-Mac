// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberNew;

public class ClimberElevateManual extends CommandBase {
  /** Creates a new ClimberElevateManual. */
  private final ClimberNew m_climber;
  public ClimberElevateManual(ClimberNew Climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climber = Climber;
    addRequirements(Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.m_arcade2.getRawAxis(1) > 0.5 ){
      m_climber.climberElevate(.85);
    } else if(RobotContainer.m_arcade2.getRawAxis(1) < -0.5 ) {
      m_climber.climberElevate(-.85);
    }else {
      m_climber.stopClimberElevate();
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
