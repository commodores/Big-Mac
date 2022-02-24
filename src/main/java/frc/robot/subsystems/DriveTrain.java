// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.    

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;




public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_TalonFX rightMasterMotor;
  private final WPI_TalonFX leftMasterMotor;

  private final WPI_TalonFX rightSlaveMotor;
  private final WPI_TalonFX leftSlaveMotor;

  private PigeonIMU pigeon;

  private MotorControllerGroup left_falcons;
  private MotorControllerGroup right_falcons;

  private final DifferentialDrive m_drive;

  private DifferentialDriveOdometry m_odometry;

  private SlewRateLimiter m_speedSlew = new SlewRateLimiter(6);


  public DriveTrain() {//DriveTrain Electronics
    rightMasterMotor = new WPI_TalonFX(DriveConstants.kRightMasterPort);
    rightSlaveMotor = new WPI_TalonFX(DriveConstants.kRightSlavePort);

    leftMasterMotor = new WPI_TalonFX(DriveConstants.kLeftMasterPort);
    leftSlaveMotor = new WPI_TalonFX(DriveConstants.kLeftSlavePort);

    pigeon = new PigeonIMU(DriveConstants.kPigeonPort);


  //Set Electronics To Default
    rightMasterMotor.configFactoryDefault();
    rightSlaveMotor.configFactoryDefault();

    leftMasterMotor.configFactoryDefault();
    leftSlaveMotor.configFactoryDefault();

    leftMasterMotor.setNeutralMode(NeutralMode.Brake);
    leftSlaveMotor.setNeutralMode(NeutralMode.Brake);
    rightMasterMotor.setNeutralMode(NeutralMode.Brake);
    rightSlaveMotor.setNeutralMode(NeutralMode.Brake);    
  
    leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMasterMotor.configStatorCurrentLimit(DriveConstants.TALON_CURRENT_LIMIT);
    leftSlaveMotor.configStatorCurrentLimit(DriveConstants.TALON_CURRENT_LIMIT);
    rightMasterMotor.configStatorCurrentLimit(DriveConstants.TALON_CURRENT_LIMIT);
    rightSlaveMotor.configStatorCurrentLimit(DriveConstants.TALON_CURRENT_LIMIT);   

    left_falcons = new MotorControllerGroup(leftMasterMotor, leftSlaveMotor);
    right_falcons = new MotorControllerGroup(rightMasterMotor, rightSlaveMotor);

    left_falcons.setInverted(true);
    right_falcons.setInverted(false);

    //leftMasterMotor.configOpenloopRamp(.2);
    //rightMasterMotor.configOpenloopRamp(.2);

    m_drive = new DifferentialDrive(left_falcons, right_falcons);


    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0));

    zeroSensors();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    m_odometry.update(
      Rotation2d.fromDegrees(getDirection()),
      getLeftDistance(),
      getRightDistance()
    );


    //SmartDashboard.putNumber("Left Encoder", getLeftDistance());
    //SmartDashboard.putNumber("Right Encoder", getRightDistance());
    //SmartDashboard.putNumber("Heading", getDirection());
  }

  
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left_falcons.setVoltage(leftVolts);
    right_falcons.setVoltage(rightVolts);
    m_drive.feed();
  }  

  public void curvatureDrive(double speed, double rotation, boolean quickturn){
    //m_drive.curvatureDrive(m_speedSlew.calculate(speed), m_turnSlew.calculate(rotation), quickturn);
    m_drive.curvatureDrive(m_speedSlew.calculate(speed), rotation*.8, quickturn);
  }

  

  public void resetEncoders() {
    leftMasterMotor.setSelectedSensorPosition(0);
    rightMasterMotor.setSelectedSensorPosition(0);
  }

  public void resetDirection() {
    pigeon.setFusedHeading(0);
  }
  
  public void zeroSensors() {
    resetEncoders();
    resetDirection();
    resetPose();    
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public double getDirection() {
    return Math.IEEEremainder(pigeon.getFusedHeading(), 360);
  }

  public double getLeftVoltage() {
    return leftMasterMotor.getMotorOutputVoltage();
  }

  public double getRightVoltage() {
    return rightMasterMotor.getMotorOutputVoltage();
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

   // distance in meters
   public double getLeftDistance() {
    return -1 * leftMasterMotor.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse;
  }

  public double getRightDistance() {
    return rightMasterMotor.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse;
  }

  // velocity in meters / sec
  public double getLeftVelocity() {
    return -1 * leftMasterMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse;
  }

  public double getRightVelocity() {
    return rightMasterMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse;
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setPos(double xloc, double yloc) {
    m_odometry.resetPosition(
      new Pose2d(xloc, yloc, Rotation2d.fromDegrees(0)),
      Rotation2d.fromDegrees(0)
    );
  }

  public void resetPose() {
    m_odometry.resetPosition(
      new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
      Rotation2d.fromDegrees(0)
    );
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  


}

