// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class Climber extends SubsystemBase {
  
  private final WPI_TalonFX climberElevate;

  private final DigitalInput elevateLimitSwitch;

  public Climber() {

    climberElevate = new WPI_TalonFX(Constants.ClimberConstants.kClimberElevatePort);
    climberElevate.configFactoryDefault();
    climberElevate.setNeutralMode(NeutralMode.Brake);
    climberElevate.set(ControlMode.PercentOutput, 0.0);
    climberElevate.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

  
    elevateLimitSwitch = new DigitalInput(2);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberElevate Motor", climberElevate.getSelectedSensorPosition());    
  }

  public void climberElevate(double speed){
    if(speed > 0 && getElevateLimitSwitch() && getClimberEncoder() <= 270000){
      climberElevate.set(ControlMode.PercentOutput, speed);
    } else if(speed < 0 && getClimberEncoder() >= 4500){
      climberElevate.set(ControlMode.PercentOutput, speed);
    } else {
      climberElevate.set(ControlMode.PercentOutput, 0);
    }
  }

  public void stopClimberElevate(){
    climberElevate.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetClimberElevateEncoder() {
    climberElevate.setSelectedSensorPosition(0);
  }

  public double getClimberEncoder(){
    return climberElevate.getSelectedSensorPosition();
  }

  public boolean getElevateLimitSwitch(){
    return elevateLimitSwitch.get();
  }

}
  
