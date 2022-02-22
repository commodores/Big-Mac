// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

import java.util.List;


public class RunTrajectory extends CommandBase {
private final RamseteCommand ramsete;
private Trajectory trajectory;

DifferentialDriveVoltageConstraint autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                    DriveConstants.kvVoltSecondsPerMeter,
                                    DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);
// Create config for trajectory
TrajectoryConfig config =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

// Create config for trajectory
TrajectoryConfig configBackwards =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);


/**
 * Define Paths here
 * 
 */
public Trajectory getDriveToFirstBallPath(){
    Trajectory driveToFirstBall = TrajectoryGenerator.generateTrajectory(
        // Start
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, -.75),
            new Translation2d(0, -1.75)
            //new Translation2d(-3.3, -1.75)

            

        ),
        new Pose2d(-3.6, -1.5, new Rotation2d(Math.PI )),
        config
    );
    return driveToFirstBall;
}

public Trajectory doNothingPath(){
  Trajectory doNothing = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
          new Translation2d(0,0)
      ),
      new Pose2d(0, 0, new Rotation2d(0)),
      config
  );
  return doNothing;
}

/** Creates a new AutoDrive. */
public RunTrajectory(String path) {
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.m_drivetrain);

  switch(path){
    case "firstBall":
      trajectory = getDriveToFirstBallPath();
    default:
      trajectory = doNothingPath();
}
  
  
this.ramsete = new RamseteCommand(
    trajectory,
    RobotContainer.m_drivetrain::getPose,
    new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                DriveConstants.kvVoltSecondsPerMeter,
                                DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    RobotContainer.m_drivetrain::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    // RamseteCommand passes volts to the callback
    RobotContainer.m_drivetrain::tankDriveVolts,
    RobotContainer.m_drivetrain
);


  }

@Override
  public void initialize() {
      ramsete.initialize();
  }

  @Override
  public void execute() {
      ramsete.execute();
  }

  @Override
  public void end(boolean interrupted) {
      ramsete.end(interrupted);
  }

  @Override
  public boolean isFinished() {
      return ramsete.isFinished();
  }

}