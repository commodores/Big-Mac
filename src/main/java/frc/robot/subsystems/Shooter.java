// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  // PID loop constants
  private double kF = ShooterConstants.kShooterF;
  private double kP = ShooterConstants.kShooterP;
  private double kI = ShooterConstants.kShooterI;
  private double kD = ShooterConstants.kShooterD;

  public int kI_Zone = 100;
  public int kAllowableError = 50;

  private TalonFX[] shooterMotors = {
          new TalonFX(ShooterConstants.kLeftShooterPort),
          new TalonFX(ShooterConstants.kRightShooterPort),
  };

  private final Solenoid hoodSolenoid;

  public double rpmOutput;
  public double rpmTolerance = 2.0;

  private double setpoint;


  //    public PIDController flywheelController = new PIDController(kP, kI, kD);
  //    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  public Shooter() {

      hoodSolenoid = new Solenoid(ShooterConstants.kHoodSolendoidPort);

      // Setup shooter motors (Falcons)
      for (TalonFX shooterMotor : shooterMotors) {
          shooterMotor.configFactoryDefault();
          shooterMotor.setNeutralMode(NeutralMode.Coast);
          shooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
          shooterMotor.configVoltageCompSaturation(11);
          shooterMotor.enableVoltageCompensation(true);
      }
      shooterMotors[0].setInverted(false);
      shooterMotors[1].setInverted(true);
      shooterMotors[1].follow(shooterMotors[0], FollowerType.PercentOutput);

      shooterMotors[0].config_kF(0, kF);
      shooterMotors[0].config_kP(0, kP);
      shooterMotors[0].config_kI(0, kI);
      shooterMotors[0].config_IntegralZone(0, kI_Zone);
      shooterMotors[0].config_kD(0, kD);
      shooterMotors[0].configAllowableClosedloopError(0, kAllowableError);
      shooterMotors[0].configClosedloopRamp(0);
      shooterMotors[1].configClosedloopRamp(0);
      shooterMotors[1].configOpenloopRamp(0);
  }

  public double getMotorInputCurrent(int motorIndex) {
      return shooterMotors[motorIndex].getSupplyCurrent();
  }

  public void setPower(double output) {
      shooterMotors[0].set(ControlMode.PercentOutput, output);
  }

  public void setRPM(double setpoint) {
      this.setpoint = setpoint;
  }

  public double getSetpoint() {
      return setpoint;
  }

  private void updateRPMSetpoint() {
      if (setpoint >= 0)
          shooterMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(setpoint));
      else
          setPower(0);
  }

  public void setTestRPM() {
      shooterMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
  }

  public double getTestRPM() {
      return rpmOutput;
  }

  public double getRPMTolerance() {
      return rpmTolerance;
  }

  public boolean encoderAtSetpoint(int motorIndex) {
      return (Math.abs(shooterMotors[motorIndex].getClosedLoopError()) < 100.0);
  }

  public double getRPM(int motorIndex) {
      return falconUnitsToRPM(shooterMotors[motorIndex].getSelectedSensorVelocity());
  }

  public double falconUnitsToRPM(double sensorUnits) {
      return (sensorUnits / 2048.0) * 600.0;
  }

  public double RPMtoFalconUnits(double RPM) {
      return (RPM / 600.0) * 2048.0;
  }

  public void hoodUp(){
      hoodSolenoid.set(true);
  }

  public void hoodDown(){
      hoodSolenoid.set(false);
  }

  @Override
  public void periodic() {
      updateRPMSetpoint();
      SmartDashboard.putNumber("Shooter RPM", falconUnitsToRPM(shooterMotors[0].getSelectedSensorVelocity()));
  }
}
