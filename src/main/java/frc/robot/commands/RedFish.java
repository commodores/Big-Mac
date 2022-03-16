// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedFish extends SequentialCommandGroup {
  /** Creates a new RedFish. */
  public RedFish() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ExtendIntake().withTimeout(0.1),
      new ParallelCommandGroup(
        new RunTrajectory("driveOffTarmac"),
        new RunIntake()
      ).withTimeout(2),
      new StopAutoIntake().withTimeout(0.1),
      new ParallelCommandGroup(
        new RunTrajectory("driveOnTarmac"),
        new ShootHigh().withTimeout(2)
      ),
      new ParallelCommandGroup(
        new ShootHigh(),
        new FireBalls()
      ).withTimeout(2),
      new ParallelCommandGroup(
        new RunTrajectory("driveToThirdBall"),
        new RunIntake()
      ).withTimeout(4),
      new StopAutoIntake().withTimeout(0.1),
      new ParallelCommandGroup(
        new RunTrajectory("driveToThirdBallShoot"),
        new ShootHigh().withTimeout(2)
      ),
      new ParallelCommandGroup(
        new ShootHigh(),
        new FireBalls()
      ).withTimeout(2)
    );
  }
}