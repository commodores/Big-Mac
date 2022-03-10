// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {

  private final WPI_TalonFX intake1;

  private final CANSparkMax intake2;
  private final CANSparkMax intake3;


  
  private final DigitalInput limitSwitch;
 

  private final DoubleSolenoid intakeSolenoid;


  public Intake() {
    intake1 = new WPI_TalonFX(IntakeConstants.kIntakeMotor1Port);

    intake1.setNeutralMode(NeutralMode.Brake);

    intake2 = new CANSparkMax(IntakeConstants.kIntakeMotor2Port, MotorType.kBrushless);
    intake3 = new CANSparkMax(IntakeConstants.kIntakeMotor3Port, MotorType.kBrushless);

    intake2.setIdleMode(IdleMode.kBrake);
    intake3.setIdleMode(IdleMode.kBrake);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 3);
    
    limitSwitch = new DigitalInput(0);

  }

  public void runIntake1(double speed){
    intake1.set(speed);
  }

  public void runIntake2(double speed){
    intake2.set(-speed);
  }

  public void runIntake3(double speed){
    intake3.set(speed);
  }

  public void stopIntake1(){
    intake1.set(0.0);
  }

  public void stopIntake2(){
    intake2.set(0.0);
  }

  public void stopIntake3(){
    intake3.set(0.0);
  }

  public boolean getLimitSwitch() {
		return limitSwitch.get();
	} 
  
  public void extendIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retractIntake() {
    intakeSolenoid.set(Value.kReverse);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Limit Switch", getLimitSwitch());
   
  }

}
  
