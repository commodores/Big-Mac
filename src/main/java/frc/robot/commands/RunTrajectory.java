// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
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
        .addConstraint(autoVoltageConstraint)
        .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAccel));

// Create config for trajectory
TrajectoryConfig configBackwards =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .addConstraint(new CentripetalAccelerationConstraint(DriveConstants.kMaxCentripetalAccel));


/**
 * Define Paths here
 * 
 */
public Trajectory getTestPath(){
    Trajectory testPath = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
          new Translation2d(1, 1), 
          new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    return testPath;
}

public Trajectory getDriveOffTarmacPath(){
  Trajectory driveOffTarmac = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints
      List.of(
      //  new Translation2d(1, 0)
      ),
      new Pose2d(1.75, 0, new Rotation2d(0)),
      // Pass config
      config);
  return driveOffTarmac;
}

public Trajectory getDriveOnTarmacPath(){
  Trajectory driveOnTarmac = TrajectoryGenerator.generateTrajectory(
      new Pose2d(1.75, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints
      List.of(
      //  new Translation2d(1, 0)
      ),
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass config
      configBackwards);
  return driveOnTarmac;
}

public Trajectory getThreeBallStart(){
  Trajectory threeBallStart = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
    ),
    new Pose2d(2.2, .75, new Rotation2d(0)),
    // Pass config
    config);
return threeBallStart;
}

public Trajectory getDriveOnTarmacThreeBallPath(){
  Trajectory driveOnTarmacThreeBall = TrajectoryGenerator.generateTrajectory(
      new Pose2d(2.2, .75, new Rotation2d(0)),
      // Pass through these two interior waypoints
      List.of(
        //new Translation2d(1, 0)
      ),
      new Pose2d(.7, 0, new Rotation2d(0)),
      // Pass config
      configBackwards);
  return driveOnTarmacThreeBall;
}

public Trajectory getDriveToThirdBall(){
    Trajectory driveToThirdBall = TrajectoryGenerator.generateTrajectory(
      new Pose2d(.7, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints
      List.of(
        //new Translation2d(.5, -.5)
      ),
      new Pose2d(1.92, -2, new Rotation2d(-45)),//was 1.5 for columbus 1.9 before columbus
      // Pass config
      config);
  return driveToThirdBall;
}

public Trajectory getDriveToThirdBallShoot(){
  Trajectory driveToThirdBallShoot = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1.65, -2, new Rotation2d(-45)),
    // Pass through these interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
    ),
    new Pose2d(.7, 0, new Rotation2d(-.25)),
    // Pass config
    configBackwards);
return driveToThirdBallShoot;
}

public Trajectory getDriveToTerminal(){
  Trajectory driveToTermainal = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.8, 0, new Rotation2d(-45)),
    // Pass through these interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
      //measured in meters
      //new Translation2d(1, 45)
    ),
    new Pose2d(1, 8, new Rotation2d(-45)),
    // Pass config
    configBackwards);
return driveToTermainal;
}

public Trajectory getDriveToRed(){
  Trajectory driveToRed = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
    ),
    new Pose2d(1, .6, new Rotation2d(0)),
    // Pass config
    config);
return driveToRed;
}

// Defense play to pick up red balls and throw them in hangar area
public Trajectory getDDefenseReset(){
  Trajectory driveToDDefenseReset = TrajectoryGenerator.generateTrajectory(
    new Pose2d(1, .6, new Rotation2d(0)),
    // Pass through these two interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
    ),
    new Pose2d(0.2, -2, new Rotation2d(-90)),
    // Pass config
    configBackwards);
return driveToDDefenseReset;
}

public Trajectory getDriveToRedTwo(){
  Trajectory driveToRedTwo = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.2, -2, new Rotation2d(-90)),
    // Pass through these interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
      //measured in meters
      //new Translation2d(0.2, -3)
    ),
    new Pose2d(0.4, -3.6, new Rotation2d(0)),
    // Pass config
    config);
return driveToRedTwo;
}

public Trajectory getDriveToHangarDefense(){
  Trajectory driveToHangarDefense = TrajectoryGenerator.generateTrajectory(
    new Pose2d(0.4, -3.6, new Rotation2d(0)),
    // Pass through these interior waypoints
    List.of(
      //new Translation2d(.5, -.5)
    ),
    new Pose2d(0.9, -3, new Rotation2d(0)),
    // Pass config
    config);
return driveToHangarDefense;
}

/** Creates a new AutoDrive. */
public RunTrajectory(String getPath) {
  // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.m_drivetrain);
  String path = getPath;
  if(path.equals("testPath")){
    trajectory = getTestPath();
  } else if(path.equals("driveOffTarmac")){
    trajectory = getDriveOffTarmacPath();
  } else if(path.equals("driveOnTarmac")){
    trajectory = getDriveOnTarmacPath();
  } else if(path.equals("threeBallStart")){
    trajectory = getThreeBallStart();
  } else if(path.equals("driveOnTarmacThreeBall")){
    trajectory = getDriveOnTarmacThreeBallPath();
  } else if(path.equals("driveToThirdBall")){
    trajectory = getDriveToThirdBall();
  } else if(path.equals("driveToThirdBallShoot")){
    trajectory = getDriveToThirdBallShoot();
  } else if(path.equals("driveToRed")) {
    trajectory = getDriveToRed();
  } else if(path.equals("driveToRedTwo")) {
    trajectory = getDriveToRedTwo();
  } else if(path.equals("driveToTerminal")) {
      trajectory = getDriveToTerminal();
  } else if(path.equals("driveToHangarDefense")) {
    trajectory = getDriveToHangarDefense();
  } else if(path.equals("driveToDDefenseReset")) {
    trajectory = getDDefenseReset();
  } else {
    trajectory = getDriveOffTarmacPath();
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