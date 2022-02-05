// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Climber extends SubsystemBase {
  
  private final WPI_TalonFX ClimberMotor;
  
  public Climber() {

    ClimberMotor = new WPI_TalonFX(Constants.ClimberConstants.kClimberPort);
    ClimberMotor.configFactoryDefault();
    ClimberMotor.setNeutralMode(NeutralMode.Brake);
    ClimberMotor.set(ControlMode.PercentOutput, 0.0);

    setDefaultCommand(new ClimberManual(this));

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runClimber(double speed){
    ClimberMotor.set(ControlMode.PercentOutput, speed);
  }

  public void StopClimber(){
    ClimberMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
