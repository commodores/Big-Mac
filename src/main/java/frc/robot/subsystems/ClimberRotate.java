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


public class ClimberRotate extends SubsystemBase {
  
  private final TalonSRX climberRotate;
  private final Solenoid climberSolenoid;

  private final DigitalInput rotateLimitSwitch;

  public ClimberRotate() {
  
    climberRotate = new TalonSRX(Constants.ClimberConstants.kClimberRotatePort);
    climberRotate.configFactoryDefault();
    climberRotate.setNeutralMode(NeutralMode.Brake);
    climberRotate.set(ControlMode.PercentOutput, 0.0);
    climberRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    climberRotate.config_kP(0, 0.1);
    climberRotate.configMotionCruiseVelocity(500);
    climberRotate.configMotionAcceleration(500);

    climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

    rotateLimitSwitch = new DigitalInput(1);    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberRotate Motor", climberRotate.getSelectedSensorPosition());    
  }
  
  public void climberRotate(double speed){
    if(getLockState()){
      if(speed > 0 && getRotateEncoder() <= 8500){
        climberRotate.set(ControlMode.PercentOutput, speed);
      } else if(speed < 0 && getRotateLimitSwitch() && getRotateEncoder() >= -1500){
        climberRotate.set(ControlMode.PercentOutput, speed);
      } else{
        climberRotate.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public void stopClimberRotate(){
    climberRotate.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetClimberRotateEncoder() {
    climberRotate.setSelectedSensorPosition(3055);
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

  public boolean getLockState(){
    return climberSolenoid.get();
  }

  public boolean getRotateLimitSwitch(){
    return rotateLimitSwitch.get();
  }

}
  
