// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  
  // Controllers
  private Joystick joystick1 = new Joystick(Constants.kDriverControllerPort);
  private Joystick joystick2 = new Joystick(Constants.kDriverControllerPort2);
  JoystickButton button1 = new JoystickButton(joystick1, 2);
  JoystickButton button2 = new JoystickButton(joystick1, 3);


  public RobotContainer() {
    configureButtonBindings();

    driveTrain.setDefaultCommand(
      new RunCommand(
              () ->
              driveTrain.arcadeDrive(
                    joystick1.getRawAxis(1), joystick1.getRawAxis(0), true),
                      driveTrain));
  }

  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button1.onTrue(intake.runIntake(0.1));
  //  button2.onTrue(led.maroonLED());
  }

  public void showTelemetry() {
    // Gyro
    SmartDashboard.putNumber("Gyro orientation", driveTrain.getAngle());
    SmartDashboard.putNumber("Gyro", driveTrain.getTurnRate());

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


  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
      var autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                  DriveConstants.ksVolts,
                  DriveConstants.kvVoltSecondsPerMeter,
                  DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics,
              10);

      // Create config for trajectory
      TrajectoryConfig config =
          new TrajectoryConfig(
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(DriveConstants.kDriveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint);

      // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
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
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
  }
}
