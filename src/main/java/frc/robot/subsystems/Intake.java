// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final VictorSPX intakeMotor;

  public Intake() {
    intakeMotor = new VictorSPX(IntakeConstants.kintakeMotorPort);

    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Brake);

    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }
  
  public void BallIn(){
    intakeMotor.set(ControlMode.PercentOutput, -1);
  }
  
  public void BallOut(){
    intakeMotor.set(ControlMode.PercentOutput, .6);
  }
  
  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
