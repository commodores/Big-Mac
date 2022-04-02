// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbHigh extends SequentialCommandGroup {
  /** Creates a new climbHigh. */
  public ClimbHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new UnLockClimber(),
      new ClimberOutToPosition(0),
      new ClimberUpToPosition(260000),
      new ClimberOutToPosition(7800),
      new ParallelCommandGroup(
        new ClimberDownToPosition(200000),
        new FlippersBack()
      )
      
    );
  }
}
