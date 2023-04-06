// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AprilAutonomous;
import frc.robot.commands.StupidForwardAutonomous;
import frc.robot.commands.Arm.ArmToAngle;
import frc.robot.commands.Claw.*;
import frc.robot.commands.DriveTrain.TurnToAngle;
import frc.robot.commands.Intake.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Intake.OurPneumatics;
import frc.robot.subsystems.Vision.Limelight;
import frc.robot.subsystems.Vision.Limelight.LimelightData;
import frc.robot.subsystems.Vision.Limelight.PoseEstimators;

import java.io.IOException;
import java.nio.file.Path;
import java.security.Permissions;

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
  private final Intake intake = new Intake();
   private final Claw claw = new Claw();
   private final Arm arm = new Arm();

  private final OurPneumatics pneumatic = new OurPneumatics();
 // private final ArmExtension armExtension = new ArmExtension();

  private final Limelight limelight = new Limelight();
  private LED led = new LED();

  private final NetworkTable networkTable =  NetworkTableInstance.getDefault().getTable("limelight");
 // private final PoseEstimators poseEstimators = new PoseEstimators(driveTrain);
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
  public Intake getIntake(){
    return this.intake;
  }
  public Claw getClaw(){
    return this.claw;
  }
  public Arm getArm(){
    return this.arm;
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  public RobotContainer() {
    configureButtonBindings();

    driveTrain.setDefaultCommand(
      new RunCommand(
              () ->
              driveTrain.arcadeDrive(
                    joystick1.getRawAxis(1), -joystick1.getRawAxis(0), true),
                      driveTrain));    

            
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Config Buttons
  /////////////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureButtonBindings() {
    // Joystick
    new JoystickButton(joystick1, 1).whileTrue(intake.runIntake(0.8));
    new JoystickButton(joystick1, 2).whileTrue(intake.runIntake(-0.8));

    new JoystickButton(joystick1, 5).onTrue(pneumatic.reverseIntake());


    // new JoystickButton(joystick1, 6).onTrue(new IntakeRetract(intake));



    // Button Board
    new JoystickButton(buttonBoard, 1).whileTrue(arm.rotateArm());
    new JoystickButton(buttonBoard, 2).whileTrue(arm.returnArm());

    
    new JoystickButton(buttonBoard, 3).whileTrue(claw.runClaw(0.2));
    new JoystickButton(buttonBoard, 4).whileTrue(claw.runClaw(-0.2)); // CHECK SIGN

    new JoystickButton(buttonBoard, 5).whileTrue(intake.adjustAngle(0.2));
    new JoystickButton(buttonBoard, 6).whileTrue(intake.adjustAngle(-0.2));

    new JoystickButton(buttonBoard, 7).whileTrue(intake.winchIntake(0.2));
    new JoystickButton(buttonBoard, 8).whileTrue(intake.winchIntake(-0.2));


  }

  public void showTelemetry() {
    // Gyro
   SmartDashboard.putNumber("Gyro orientation", driveTrain.getAngle());
    
    // Encoders
    SmartDashboard.putNumber("Encoder Left", driveTrain.getLeftDistance());
    SmartDashboard.putNumber("Encoder Right", driveTrain.getRightDistance());
    SmartDashboard.putNumber("Encoder Claw", claw.getEncoderPosition());
    SmartDashboard.putNumber("Encoder Intake", intake.getIntakeEncoderPos());

    SmartDashboard.putNumber("Arm angle", arm.getAngle());

   
    // Limelight
    System.out.println( LimelightData.getX());
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

     //eturn ramseteCommand();
    // return new AprilAutonomous(driveTrain);
    // return driveTrain.stupidAutoDriveForwardOutake1Command(driveTrain, intake, 0.1);

      return arm.rotateArm().andThen(claw.runClaw(0.2)).andThen(driveTrain.stupidAutoDriveBackwardsCommand(driveTrain, -0.2));
  }
}
