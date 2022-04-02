// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity!
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int kLeftMasterPort = 13;
        public static final int kLeftSlavePort = 14;

        public static final int kRightMasterPort = 12;
        public static final int kRightSlavePort = 11;

        public static final int kPigeonPort = 22;

        public static final int driveTimeout = 30;

        public static final int kEncoderCPR = 2048; //https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html
        public static final double kWheelDiameterMeters = 0.1016; //4 inches
        public static final double kGearReduction = 7;
        public static final double kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR) / kGearReduction; 
        public static final double kWheelDistancePerPulse = kEncoderDistancePerPulse/ kGearReduction; //DISTANCE PER PULSE OF WHEEL= (OUTER CIRCUMFERENCE OF WHEEL)/(ENCODER CPR*GEAR REDUCTION)

        public static final double ksVolts = 0.68824;
        public static final double kvVoltSecondsPerMeter = 2.389;
        public static final double kaVoltSecondsSquaredPerMeter = 0.4162;
        public static final double kPDriveVel = .5; //2.24;   // 2.29

        public static final double kTrackwidthMeters = 0.648716;//CAD //.5715 Tape //0.59825 From Char Tool
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kDriveTrainGain = .015;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxCentripetalAccel = 1.5;


        public static final boolean kGyroReversed = true;

        public static StatorCurrentLimitConfiguration TALON_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 36.5, 36.5, 0.25);
        

    }

    public static final class ShooterConstants {
        public static final int kRightShooterPort = 15;
        public static final int kLeftShooterPort = 16;

        public static final double kShooterVoltageRampRate = 0.2;

        public static final double kShooterP = 0.0055;
        public static final double kShooterI = 0; //1
        public static final double kShooterD = 0; //5
        public static final double kShooterF = 0.04; // 0.058

        public static final int kAllowableError = 50;
        public static final int kPIDLoopRate = 10; //In ms
        public static final int kMaxIntegralAccumulator = 1000;
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kDriverController2Port = 1;
        public static final int kDriverController3Port = 2;
        public static final int kDriverController4Port = 3;
    }

    public static final class IntakeConstants{
        public static final int kIntakeSolenoidPort = 0;
        public static final int kIntakeMotor1Port = 17;
        public static final int kIntakeMotor2Port = 18;
        public static final int kIntakeMotor3Port = 19;

    }

    public static final class ClimberConstants{
        public static final int kClimberElevatePort = 20;
       public static final int kClimberRotatePort = 21;
    }
}

