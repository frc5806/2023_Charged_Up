// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // TODO: Update all constants + find new later
    public static final class DriveConstants {
        public static final double kRobotLength = 0;
        public static final double kRobotWidth = 0;

        public static final int kLeftMotor1Port = 17;
        public static final int kLeftMotor2Port = 4;
        public static final int kLeftMotor3Port = 20;

        public static final int kRightMotor1Port = 1;
        public static final int kRightMotor2Port = 8;
        public static final int kRightMotor3Port = 18;

        public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
        public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTurnP = 3.1827E-07;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kTurnToleranceDeg = 7;
        public static final double kTurnRateToleranceDegPerS = 10;

        // TODO: Charaterize drive with sysid
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double ksVolts = 0.11858;
        public static final double kvVoltSecondsPerMeter = 1.339;
        public static final double kaVoltSecondsSquaredPerMeter = 0.20898;

        public static final double kPDriveVel = 8.5;

        // Ultrasonic sensor
        public static final int kUltrasonicPort = 0;
        public static final int kUltrasonicPort1 = 1;

    // Encoder ticks to feet
    public static final double kDriveTickToFeet = (1.0/4096) * 6 * (Math.PI/12);
    public static final double kDriveTickToFeetToMeters = kDriveTickToFeet * 0.3048;
    // private static final double kArmTickToDegrees = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double stupidEstimatedDistanceFeet = 1;
        public static final double backUpFeet = -2;

    }

    public static final int ledPort = 8;
    public static final int ledLength = 88;

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort1 = 7; // 8
        public static final int kIntakeMotorPort2 = 11;
         public static final int kIntakeMotorPort3 = 10;
        // public static final int kIntakeMotorPort4 = 11;

        public static final int intakePosClose = -7;
        public static final int intakePosOpen = 7;

    }

    public static final class ArmConstants {
        public static final int kArmMotorPort = 6;
        public static final int kArmMotorPort1 = 3; // 18
        public static final int kArmMotorPort2 = 13;
        public static final int kArmMotorPort3 = 2;

        public static final int countsPerRev = 42;

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

    }

    public static final class ClawConstants {
        public static final int kClawMotorPort = 5;

        public static final double clawPosOpen = -1;
        public static final double clawPosClose = 1;
    }

    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;

    public static final class VisionConstants {
        // Adjust please
        public static final double kvisionCameraX = Units.inchesToMeters(0);
        public static final double kvisionCameraY = Units.inchesToMeters(0);
        public static final double kvisionCameraZ = Units.inchesToMeters(0);
        public static final Rotation3d kvisionCameraRotation = new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), Units.degreesToRadians(0.0));
        
        // Configure this
        public static final int klimelightPipelineAprilTag = 1;
        public static final int klimelightPipelineVision = 0;

    }

    public static final class PneumaticsConstants {
        public static final int kPneumaticsPortL = 1;
        public static final int kPneumaticsPortR = 2;
    }

    
}