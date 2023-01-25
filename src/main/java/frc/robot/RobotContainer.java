// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.*;
import frc.robot.commands.DrivetrainDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain = new DriveTrain();
  private final Intake intake = new Intake();
  private NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("NetworkTable");

  private LED led = new LED();
  

 // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private Joystick joystick1 = new Joystick(Constants.kDriverControllerPort);
  private Joystick joystick2 = new Joystick(Constants.kDriverControllerPort2);
  JoystickButton button1 = new JoystickButton(joystick1, 2);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // configureButtonBindings();

    // Set drive as default
    driveTrain.setDefaultCommand(
      new RunCommand(
              () ->
              driveTrain.arcadeDrive(
                    joystick1.getRawAxis(0), joystick1.getRawAxis(1), true),
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
  }

  public void showTelemetry() {
    SmartDashboard.putNumber("Gyro orientation", driveTrain.getAngle());
  }

  public void runLED() {
    led.run();
    led.changeMode();

  }
  /** 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
