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
import edu.wpi.first.wpilibj.DigitalOutput;
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


  
  private final DigitalInput toplimitSwitch;
  private final DigitalInput bottomlimitSwitch;

  private final DoubleSolenoid intakeSolenoid;

  //private final Solenoid intakeSolenoid;

  public Intake() {

    //intakeSolenoid = new Solenoid(IntakeConstants.kIntakeSolenoidPort);
    intake1 = new WPI_TalonFX(IntakeConstants.kIntakeMotor1Port);

    intake1.setNeutralMode(NeutralMode.Brake);

    intake2 = new CANSparkMax(IntakeConstants.kIntakeMotor2Port, MotorType.kBrushless);
    intake3 = new CANSparkMax(IntakeConstants.kIntakeMotor3Port, MotorType.kBrushless);

    intake2.setIdleMode(IdleMode.kBrake);
    intake3.setIdleMode(IdleMode.kBrake);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
    
    toplimitSwitch = new DigitalInput(0);
    bottomlimitSwitch = new DigitalInput(1);


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

  public boolean getUpper() {
		return toplimitSwitch.get();
	}

  public boolean getLower() {
		return bottomlimitSwitch.get();
	}
  
 public void extendIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void retractIntake() {
    intakeSolenoid.set(Value.kForward);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Limit Switch",  getUpper());
    SmartDashboard.putBoolean("Bottom Limit Swicth", getLower());
  }

}
  
