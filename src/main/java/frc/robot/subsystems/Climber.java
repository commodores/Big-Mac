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
  private final TalonSRX climberRotate;
  private final Solenoid climberSolenoid;

  private final DigitalInput rotateLimitSwitch;
  private final DigitalInput elevateLimitSwitch;

  public Climber() {

    climberElevate = new WPI_TalonFX(Constants.ClimberConstants.kClimberElevatePort);
    climberElevate.configFactoryDefault();
    climberElevate.setNeutralMode(NeutralMode.Brake);
    climberElevate.set(ControlMode.PercentOutput, 0.0);
    climberElevate.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    climberElevate.config_kP(0, 0.1);
    climberElevate.configMotionCruiseVelocity(5000);
    climberElevate.configMotionAcceleration(5000);
  
    climberRotate = new TalonSRX(Constants.ClimberConstants.kClimberRotatePort);
    climberRotate.configFactoryDefault();
    climberRotate.setNeutralMode(NeutralMode.Brake);
    climberRotate.set(ControlMode.PercentOutput, 0.0);
    climberRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    climberRotate.config_kP(0, 0.1);
    climberRotate.configMotionCruiseVelocity(5000);
    climberRotate.configMotionAcceleration(5000);

    climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);

    rotateLimitSwitch = new DigitalInput(1);
    elevateLimitSwitch = new DigitalInput(2);
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberRotate Motor", climberRotate.getSelectedSensorPosition());
    SmartDashboard.putNumber("ClimberElevate Motor", climberElevate.getSelectedSensorPosition());
    
  }
  public void climberElevate(double speed){
    if(speed > 0 && getClimberEncoder() <= 270000 && getElevateLimitSwitch()){
      climberElevate.set(ControlMode.PercentOutput, speed);
    } else if(speed < 0 && getClimberEncoder() >= 10000){
      climberElevate.set(ControlMode.PercentOutput, speed);
    } else {
      climberElevate.set(ControlMode.PercentOutput, 0);
    }
  }

  public void elevateMagic(double position){
    climberElevate.set(ControlMode.MotionMagic, position);
  }

  public void climberRotate(double speed){
    if(getLockState()){
      if(speed > 0 && getRotateEncoder() <= 7865){
        climberRotate.set(ControlMode.PercentOutput, speed);
      } else if(speed < 0 && getRotateLimitSwitch() && getRotateEncoder() >= -250){
        climberRotate.set(ControlMode.PercentOutput, speed);
      } else{
        climberRotate.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public void rotateMagic(double position){
    climberRotate.set(ControlMode.MotionMagic, position);
  }

  public void stopClimberElevate(){
    climberElevate.set(ControlMode.PercentOutput, 0.0);
  }

  public void stopClimberRotate(){
    climberRotate.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetClimberEncoders() {
    climberElevate.setSelectedSensorPosition(0);
    climberRotate.setSelectedSensorPosition(3729);
  }

  public void climberLock(){
    climberSolenoid.set(false);
  }

  public void climberUnlock(){
    climberSolenoid.set(true);
  }

  public double getRotateEncoder(){
    return climberRotate.getSelectedSensorPosition();
  }
  public double getClimberEncoder(){
    return climberElevate.getSelectedSensorPosition();
  }

  public boolean getLockState(){
    return climberSolenoid.get();
  }

  public boolean getRotateLimitSwitch(){
    return rotateLimitSwitch.get();
  }

  public boolean getElevateLimitSwitch(){
    return elevateLimitSwitch.get();
  }

}
  
