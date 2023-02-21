// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.Constants.AutoConstants;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  // Subsystems
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();

  private final Limelight limelight = new Limelight();
  private LED led = new LED();

  private final NetworkTable networkTable =  NetworkTableInstance.getDefault().getTable("limelight");
  // Commands
  private final TurnToAngle turnToAngle = new TurnToAngle(10, driveTrain);
  
  // Controllers
  private Joystick joystick1 = new Joystick(Constants.kDriverControllerPort);
  private Joystick joystick2 = new Joystick(Constants.kDriverControllerPort2);
  JoystickButton button1 = new JoystickButton(joystick1, 1);
  JoystickButton button2 = new JoystickButton(joystick1, 2);

  public RobotContainer() {
    configureButtonBindings();

    driveTrain.setDefaultCommand(
      new RunCommand(
              () ->
              driveTrain.arcadeDrive(
                    joystick1.getRawAxis(1), -joystick1.getRawAxis(0), true),
                      driveTrain));
  }

  private void configureButtonBindings() {
    button1.whileTrue(intake.runIntake(0.5));
    button2.onTrue(turnToAngle);
  //  button2.onTrue(led.maroonLED());
  }

  public void showTelemetry() {
    // Gyro
    SmartDashboard.putNumber("Gyro orientation", driveTrain.getAngle());
    SmartDashboard.putNumber("Gyro", driveTrain.getTurnRate());

    // Encoders
    SmartDashboard.putNumber("Encoder Left value", driveTrain.getLeftDistance());
    SmartDashboard.putNumber("Encoder Right value", driveTrain.getRightDistance());

    // Limelight
    limelight.update();
    SmartDashboard.putNumber("Limelight X", limelight.getX());
    SmartDashboard.putNumber("Limelight Y", limelight.getX());
    SmartDashboard.putNumber("Limelight Area", limelight.getX());
  }

  public void runLED() {
//    led.run();
    // led.changeMode();
  }


   public Command getAutonomousCommand() {
      var autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                  DriveConstants.ksVolts,
                  DriveConstants.kvVoltSecondsPerMeter,
                  DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics,
              2);

      TrajectoryConfig config =
          new TrajectoryConfig(
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics)
              .addConstraint(autoVoltageConstraint);

      // An example trajectory to follow.  All units in meters.
      //  Trajectory exampleTrajectory =
      //   TrajectoryGenerator.generateTrajectory(
      //       // Start at the origin facing the +X direction
      //       new Pose2d(0, 0, new Rotation2d(0)),
      //       // Pass through these two interior waypoints, making an 's' curve path
      //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      //       // List.of(new Translation2d(0, 0), new Translation2d(2, -1)),

      //       // End 3 meters straight ahead of where we started, facing forward
      //       new Pose2d(3, 0, new Rotation2d(0)),
      //       // Pass config
      //       config); 

    String trajectoryJSON = "paths/YourPath.wpilib.json";
    Trajectory trajectory = new Trajectory();
          
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
