// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kLeftMotor3Port = 3;

        public static final int kRightMotor1Port = 5;
        public static final int kRightMotor2Port = 6;
        public static final int kRightMotor3Port = 7;

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
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final int ledPort = 8;
    public static final int ledLength = 88;

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort1 = 8;
        public static final int kIntakeMotorPort2 = 9;
        public static final int kIntakeMotorPort3 = 10;
        public static final int kIntakeMotorPort4 = 11;
    }

    public static final class ClawConstants {
        public static final int kClawMotorPort = 4;
    }

    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
}