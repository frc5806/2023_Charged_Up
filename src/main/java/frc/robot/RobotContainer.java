// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AprilAutonomous;
import frc.robot.commands.Arm.ArmToAngle;
import frc.robot.commands.Claw.*;
import frc.robot.commands.DriveTrain.TurnToAngle;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Limelight.LimelightData;
import frc.robot.subsystems.Limelight.PoseEstimators;

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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Subsystems
  /////////////////////////////////////////////////////////////////////////////////////////////////////////  
  private final DriveTrain driveTrain = new DriveTrain();
  // private final Intake intake = new Intake();
   private final Claw claw = new Claw();
 //  private final Arm arm = new Arm();
 // private final ArmExtension armExtension = new ArmExtension();

  private final Limelight limelight = new Limelight();
  private LED led = new LED();

  private final NetworkTable networkTable =  NetworkTableInstance.getDefault().getTable("limelight");
  private final PoseEstimators poseEstimators = new PoseEstimators(driveTrain);
  // Commands
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // private final Command startArm = Commands.runOnce(armExtension::enable, armExtension);
  // private final Command stopArm = Commands.runOnce(armExtension::disable, armExtension);
 // private final Command armToAngle = new ArmToAngle(10, arm);
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Controllers
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  private Joystick joystick1 = new Joystick(Constants.kDriverControllerPort);
  private Joystick buttonBoard = new Joystick(Constants.kDriverControllerPort2);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Get Methods
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // public Intake getIntake(){
  //   return this.intake;
  // }
  // public Claw getClaw(){
  //   return this.claw;
  // }
  // public Arm getArm(){
  //   return this.arm;
  // }

  public RobotContainer() {
    configureButtonBindings();

    // driveTrain.setDefaultCommand(
    //   new RunCommand(
    //           () ->
    //           driveTrain.arcadeDrive(
    //                 joystick1.getRawAxis(1), -joystick1.getRawAxis(0), true),
    //                   driveTrain));

              driveTrain.setDefaultCommand(
                    new RunCommand(
                        () -> {
                        new JoystickButton(buttonBoard, 3).onTrue(new ClawPos(-0.25, claw));
                        new JoystickButton(buttonBoard, 4).onTrue(new ClawPos(0.25, claw));
                      }));
                        // new JoystickButton(buttonBoard, 4).onTrue(new ClawPos(0.25, claw)))));
    

            
  }

  private void configureButtonBindings() {
    // new JoystickButton(buttonBoard, 1).whileTrue(intake.runIntake(0.5));
    new JoystickButton(buttonBoard, 2).onTrue(new TurnToAngle(10, driveTrain));
    new JoystickButton(buttonBoard, 3).onTrue(new ClawPos(-0.25, claw));
    new JoystickButton(buttonBoard, 4).onTrue(new ClawPos(0.25, claw));
  }

  public void showTelemetry() {
    // Gyro
    // SmartDashboard.putNumber("Gyro orientation", driveTrain.getAngle());
    // SmartDashboard.putNumber("Gyro", driveTrain.getTurnRate());
    // // Encoders
    // SmartDashboard.putNumber("Encoder Left value", driveTrain.getLeftDistance());
    // SmartDashboard.putNumber("Encoder Right value", driveTrain.getRightDistance());

    SmartDashboard.putNumber("Encoder value Claw", claw.getEncoderPosition());
    // Limelight
   

    System.out.println( LimelightData.getX());


    SmartDashboard.putNumber("Ultrasonic Distance", driveTrain.getUltrasonicDistance());
    SmartDashboard.putNumber("Ultrasonic Distance 1", driveTrain.getUltrasonicDistance1());

    SmartDashboard.putNumber("LimelightX", LimelightData.getX());
    SmartDashboard.putNumber("LimelightY", LimelightData.getY());


    // NetworkTableEntry tx = networkTable.getEntry("tx");
    // System.out.println(tx.getDouble(0));
    //  double tagID = networkTable.getDefault().getTable("limelight")
    // private final NetworkTable tagID = 

    // double[] id = networkTable.getEntry("botpose").getDoubleArray(new double[6]);
    // for (int i = 0; i<id.length; i++ ) {
    //   System.out.println("i value: " + Integer.toString(i) + ". Value:  " + id[i]);
    // }

    // networkTable.getEntry("ledMode").setNumber(3);
    // System.out.println(networkTable.getEntry("tid").getDoubleArray(new double[6])[0]);
    
    LimelightData.update();
    LimelightData.dataTest();




    // Double tagID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);
    // System.out.println();
    // System.out.println("print");
  }

  public void runLED() {
//    led.run();
    // led.changeMode();
  }


   public Command getAutonomousCommand() {
    //   var autoVoltageConstraint =
    //       new DifferentialDriveVoltageConstraint(
    //           new SimpleMotorFeedforward(
    //               DriveConstants.ksVolts,
    //               DriveConstants.kvVoltSecondsPerMeter,
    //               DriveConstants.kaVoltSecondsSquaredPerMeter),
    //           DriveConstants.kDriveKinematics,
    //           2);

    //   TrajectoryConfig config =
    //       new TrajectoryConfig(
    //               AutoConstants.kMaxSpeedMetersPerSecond,
    //               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //           .setKinematics(DriveConstants.kDriveKinematics)
    //           .addConstraint(autoVoltageConstraint);

    //   // An example trajectory to follow.  All units in meters.
    //   //  Trajectory exampleTrajectory =
    //   //   TrajectoryGenerator.generateTrajectory(
    //   //       // Start at the origin facing the +X direction
    //   //       new Pose2d(0, 0, new Rotation2d(0)),
    //   //       // Pass through these two interior waypoints, making an 's' curve path
    //   //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //   //       // List.of(new Translation2d(0, 0), new Translation2d(2, -1)),

    //   //       // End 3 meters straight ahead of where we started, facing forward
    //   //       new Pose2d(3, 0, new Rotation2d(0)),
    //   //       // Pass config
    //   //       config); 

    // String trajectoryJSON = "paths/Test_Auto.wpilib.json";
    // Trajectory trajectory = new Trajectory();
          
    // try {
    //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }

    // RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //         trajectory,
    //         driveTrain::getPose,
    //         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         driveTrain::getWheelSpeeds,
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         driveTrain::tankDriveVolts,
    //         driveTrain);

    

    // // Reset odometry to the starting pose of the trajectory.
    // driveTrain.resetOdometry(trajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    // return     PoseEstimators.updatePoseEstimator(driveTrain); // NOT A COMMAND

    // return ramseteCommand();
    return new AprilAutonomous(driveTrain);


  }
}
