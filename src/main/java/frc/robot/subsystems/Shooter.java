// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class Shooter extends SubsystemBase {
  /**
   * Initializes the SparkMAX motor controller, assigns it to the CAN address
   * specified, and sets it to the NEO Brushless Motor.
   */
  private final CANSparkMax shooterMAXLeft = new CANSparkMax(ShooterConstants.kshooterMotor1Port,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax shooterMAXRight = new CANSparkMax(ShooterConstants.kshooterMotor2Port,
      CANSparkMaxLowLevel.MotorType.kBrushless);
  private CANEncoder m_encoder;
  /**
   * The built-in PID controller provided by the Spark MAX motor controller.
   */
  private final CANPIDController shooterPIDLeft;
  /**
   * The target velocity of the NEO Brushless Motor.
   */
  private double shooterSetpoint = 3750;
  /**
   * The Proportial Gain of the SparkMAX PIDF controller The weight of the
   * proportional path against the differential and integral paths is controlled
   * by this value.
   */
  private final double kP = .00065;
  /**
   * The Integral Gain of the SparkMAX PIDF controller The weight of the integral
   * path against the proportional and differential paths is controlled by this
   * value.
   */
  private final double kI = 0;
  /**
   * The Differential Gain of the SparkMAX PIDF controller. The weight of the
   * differential path against the proportional and integral paths is controlled
   * by this value.
   */
  private final double kD = 0;
  /**
   * The Integral Zone of the SparkMAX PIDF controller. The integral accumulator
   * will reset once it hits this value.
   */
  private final double kIz = 0;
  /**
   * The Feed-Forward Gain of the SparkMAX PIDF controller. The weight of the
   * feed-forward loop as compared to the PID loop is controlled by this value.
   */
  private final double kFF = 0.000177;
  /**
   * Scales the output of the SparkMAX PIDF controller.
   */
  private final double maxOutput = 1.0;
  private final double minOutput = -1.0;
  private final double maxRPM = 5700;
  /**
   * The maximum current the motor controller is allowed to feed to the motor, in
   * amps.
   */
  private final int currentLimit = 40;

  public Shooter() {
    shooterMAXLeft.restoreFactoryDefaults();
    shooterMAXRight.restoreFactoryDefaults();

    shooterPIDLeft = shooterMAXLeft.getPIDController();
    m_encoder = shooterMAXLeft.getEncoder();

    // Applies the previously-declared values to the PIDF controller.
    shooterPIDLeft.setP(kP);
    shooterPIDLeft.setI(kI);
    shooterPIDLeft.setD(kD);
    shooterPIDLeft.setIZone(kIz);
    shooterPIDLeft.setFF(kFF);
    shooterPIDLeft.setOutputRange(minOutput, maxOutput);
    // Sets the shooter motor to coast so that subsequent shots don't have to rev up
    // from 0 speed.
    shooterMAXLeft.setIdleMode(IdleMode.kCoast);
    shooterMAXRight.setIdleMode(IdleMode.kCoast);
    shooterMAXLeft.setSmartCurrentLimit(currentLimit);
    shooterMAXRight.setSmartCurrentLimit(currentLimit);
    // Sets the left shooter motor to follow the right motor, and be inverted.
    shooterMAXRight.follow(shooterMAXLeft, true);
  }

  /**
   * Passes a preset velocity to the SparkMAX PIDF controller and lets it manage
   * the NEO's velocity. Intended to be called when a button is pressed.
   */
  public void shoot(double setPoint) {
    shooterPIDLeft.setReference(setPoint, ControlType.kVelocity);
  }

  /**
   * Stops the shooter motor. Note: the NEO is set to Coast. Intended to be called
   * when a button is released.
   */
  public void stopShooter() {
    shooterMAXLeft.stopMotor();
  }

  public void setSetpoint(double newPoint) {
    shooterSetpoint = newPoint;
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("SetPoint", shooterSetpoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
  }
}
