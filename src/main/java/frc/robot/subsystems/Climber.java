// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {public LeftElevator() {

    leftElevator = new TalonSRX(ElevatorConstants.kelevatorMotorLeftPort);

    leftElevator.configFactoryDefault();
    leftElevator.setNeutralMode(NeutralMode.Brake);
    leftElevator.set(ControlMode.PercentOutput, 0.0);

    setDefaultCommand(new ElevatorManual(this));
  }

  public void ElevatorUp(){
    leftElevator.set(ControlMode.PercentOutput, -1);
  }

  public void ElevatorDown(){
    leftElevator.set(ControlMode.PercentOutput, 1);
  }

  public void StopElevator(){
    leftElevator.set(ControlMode.PercentOutput, 0.0);
  }
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
