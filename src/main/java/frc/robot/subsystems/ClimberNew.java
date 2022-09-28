// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class ClimberNew extends SubsystemBase {
  /** Creates a new ClimberNew.
  New Climber is one talon that will move up and down.
  */

  private final WPI_TalonFX climberNew;
  private final DoubleSolenoid climberSolenoid;

  public ClimberNew() {
    
    climberNew = new WPI_TalonFX(Constants.ClimberConstants.kClimberElevatePort);
    climberNew.configFactoryDefault();
    climberNew.setNeutralMode(NeutralMode.Brake);
    climberNew.set(ControlMode.PercentOutput, 0.0);
    climberNew.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2,4);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberNew", climberNew.getSelectedSensorPosition());
  }

  public void climberElevate(double speed) {
    if (getLockState()) {
      /*if(speed < 0 && getClimberEncoder() <= 270000){
        climberNew.set(ControlMode.PercentOutput, speed);
      } else if(speed < 0 && getClimberEncoder() >= 4500){
        climberNew.set(ControlMode.PercentOutput, speed);
      } else {
        climberNew.set(ControlMode.PercentOutput, 0);
      }*/

      if(speed < 0){
        climberNew.set(ControlMode.PercentOutput, speed);
      } else if(speed > 0){
        climberNew.set(ControlMode.PercentOutput, speed);
      } else {
        climberNew.set(ControlMode.PercentOutput, 0);
      }
    }
  }

  public boolean getLockState() {
    var state = climberSolenoid.get();
    if(state == Value.kForward){
      return true;
    } else {
      return false;
    }
  }

  public void climberLock(){
    climberSolenoid.set(Value.kReverse);;
  }

  public void climberUnlock(){
    climberSolenoid.set(Value.kForward);
  }

  public void resetClimberElevateEncoder() {
    climberNew.setSelectedSensorPosition(0);
  }

  public double getClimberEncoder(){
    return climberNew.getSelectedSensorPosition();
  }

  public void stopClimberElevate() {
    climberNew.set(ControlMode.PercentOutput, 0.0);
  }
}
