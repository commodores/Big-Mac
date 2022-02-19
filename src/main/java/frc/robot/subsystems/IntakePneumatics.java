// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class IntakePneumatics extends SubsystemBase {
  private final Solenoid intakeSolenoid;
  /** Creates a new IntakePneumatics. */
  public IntakePneumatics() {
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
  }

  public void extendIntake() {
    intakeSolenoid.set(false);
  }

  public void retractIntake() {
    intakeSolenoid.set(true);
  }
}
