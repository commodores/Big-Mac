// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Climber extends SubsystemBase {
  
  private final WPI_TalonFX ClimberElevate;
  private final TalonSRX ClimberRotate;

  public Climber() {

     ClimberElevate = new WPI_TalonFX(Constants.ClimberConstants.kClimberElevatePort);
    ClimberElevate.configFactoryDefault();
    ClimberElevate.setNeutralMode(NeutralMode.Brake);
    ClimberElevate.set(ControlMode.PercentOutput, 0.0);
    ClimberElevate.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  
   ClimberRotate = new TalonSRX(Constants.ClimberConstants.kClimberRotatePort);
    ClimberRotate.configFactoryDefault();
    ClimberRotate.setNeutralMode(NeutralMode.Brake);
    ClimberRotate.set(ControlMode.PercentOutput, 0.0);
    ClimberRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("ClimberRotate Motor", ClimberRotate.getSelectedSensorPosition());
    SmartDashboard.putNumber("ClimberElevate Motor", ClimberElevate.getSelectedSensorPosition());
  }
  public void climberElevate(double speed){
    ClimberElevate.set(ControlMode.PercentOutput, speed);  
  }

  public void climberRotate(double speed){
    ClimberRotate.set(ControlMode.PercentOutput, speed); 
  }


  public void stopClimberElevate(){
    ClimberElevate.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopClimberRotate(){
    ClimberRotate.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetEncoders() {
    ClimberElevate.setSelectedSensorPosition(0);
    ClimberRotate.setSelectedSensorPosition(0);
  }


}
