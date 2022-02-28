// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class DriveManual extends CommandBase {
  /** Creates a new DriveManual. */
  private final DriveTrain m_drivetrain;
  public DriveManual(DriveTrain DriveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = DriveTrain;
    addRequirements(DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftTrigger = RobotContainer.m_driverController.getRawAxis(2);
    double rightTrigger = RobotContainer.m_driverController.getRawAxis(3);
    double speed = rightTrigger - leftTrigger;

    double rotation = RobotContainer.m_driverController.getRawAxis(0);

    boolean quickTurn = false;//speed > -0.1 && speed < 0.1;

    m_drivetrain.curvatureDrive(speed, rotation, quickTurn);

    //m_drivetrain.arcadeDrive(speed, rotation*.5);

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