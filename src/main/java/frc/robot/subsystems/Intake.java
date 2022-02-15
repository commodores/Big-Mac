// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final WPI_TalonFX intake1;

  private final CANSparkMax intake2;
  private final CANSparkMax intake3;

  private final Solenoid intakeSolenoid;

  //private final Solenoid intakeSolenoid;

  public Intake() {

    //intakeSolenoid = new Solenoid(IntakeConstants.kIntakeSolenoidPort);
    intake1 = new WPI_TalonFX(IntakeConstants.kIntakeMotor1Port);

    intake2 = new CANSparkMax(IntakeConstants.kIntakeMotor2Port, MotorType.kBrushless);
    intake3 = new CANSparkMax(IntakeConstants.kIntakeMotor3Port, MotorType.kBrushless);

    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);


  }


  public void runIntake(double speed){
    intake1.set(speed);
    intake2.set(-speed);
    intake3.set(speed);
  }

  public void stopIntake(){
    intake1.set(0.0);
    intake2.set(0.0);
    intake3.set(0.0);
  }

  public void extendIntake() {
    intakeSolenoid.set(false);
  }

  public void retractIntake() {
    intakeSolenoid.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
