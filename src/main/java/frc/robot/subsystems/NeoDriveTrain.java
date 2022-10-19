// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveManual;

public class NeoDriveTrain extends SubsystemBase {

  CANSparkMax leftMaster = new CANSparkMax(DriveConstants.kLeftMasterPort,MotorType.kBrushless);
  CANSparkMax leftSlave0 = new CANSparkMax(DriveConstants.kLeftSlavePort,MotorType.kBrushless);

  CANSparkMax rightMaster = new CANSparkMax(DriveConstants.kRightMasterPort,MotorType.kBrushless);
  CANSparkMax rightSlave0 = new CANSparkMax(DriveConstants.kRightSlavePort,MotorType.kBrushless);
  
  RelativeEncoder leftEncoder = leftMaster.getEncoder();
  RelativeEncoder rightEncoder = rightMaster.getEncoder();

  PigeonIMU pigeon = new PigeonIMU(DriveConstants.kPigeonPort);

  DifferentialDrive m_drive;
  
  private double[] yawPitchRoll = new double[3];
  private double[] xyz_dps = new double[3];
  PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
	PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();

  private final DifferentialDriveOdometry m_odometry;
    
  public NeoDriveTrain() {

    leftMaster.restoreFactoryDefaults();
    leftSlave0.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    rightSlave0.restoreFactoryDefaults();

    leftSlave0.follow(leftMaster);
    rightSlave0.follow(rightMaster);

    rightMaster.setOpenLoopRampRate(.45);
    leftMaster.setOpenLoopRampRate(.45);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    rightMaster.setIdleMode(IdleMode.kBrake);
    leftMaster.setIdleMode(IdleMode.kBrake);

    leftMaster.setInverted(true);
    //rightMaster.setInverted(true);

    m_drive = new DifferentialDrive(leftMaster, rightMaster);
    
    m_drive.setSafetyEnabled(false);

    //resetEncoders();

    leftEncoder.setPositionConversionFactor(DriveConstants.drivetrainEncoderConversionFactor);
    leftEncoder.setVelocityConversionFactor(DriveConstants.drivetrainEncoderConversionFactor);

    rightEncoder.setPositionConversionFactor(DriveConstants.drivetrainEncoderConversionFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.drivetrainEncoderConversionFactor);

    m_odometry = new DifferentialDriveOdometry(new Rotation2d(0));

    //setDefaultCommand(new DriveManual(this));

  }

  @Override
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(getDirection()),
      getLeftDistance(),
      getRightDistance()
    );
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(rightVolts);
  }  

  public void curvatureDrive(double speed, double rotation, boolean quickturn){
    m_drive.curvatureDrive(speed, rotation * .6, quickturn);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    
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
    return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
  }

  public double getDirection() {
    return Math.IEEEremainder(pigeon.getFusedHeading(), 360);
  }

  public double getLeftSpeed() {
    return leftEncoder.getVelocity();
  }

  public double getRightSpeed() {
    return rightEncoder.getVelocity();
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

   // distance in meters
   public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  public double getRightDistance() {
    return rightEncoder.getPosition();
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

  public double getCurrentAngle() {
    return pigeon.getAbsoluteCompassHeading();
  }


}


  