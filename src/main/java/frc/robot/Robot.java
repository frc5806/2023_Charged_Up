// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.OurPneumatics;
// import frc.robot.subsystems.Intake.OurPneumatics;
import frc.robot.subsystems.Vision.Limelight.LimelightMode;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  public double autoStart;


  public double getAutoStartTime(){
    return autoStart;
  }
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    PortForwarder.add(8888, "wpilibpi.local", 80);
    PortForwarder.add(9999, "limelight.local", 99);

  }


  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (Timer.getFPGATimestamp() % 5 == 0) {
    double time = Timer.getFPGATimestamp();
    System.out.println("Time: ");
    System.out.println(time);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
      System.out.println("auton init");
      
    } 
    autoStart = Timer.getFPGATimestamp();


  }

  @Override
  public void autonomousPeriodic() {
    OurPneumatics.enableCompressor();

    // if (autonomousCommand != null) {
    //   autonomousCommand.cancel();
    // }


  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    LimelightMode.setPipelineApril();

  }

  @Override
  public void teleopPeriodic() {
    robotContainer.showTelemetry();
    robotContainer.runLED();

    OurPneumatics.enableCompressor();
    // System.out.println("iu3fwe");
    // double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    // for (int i = 0; i<id.length; i++ ) {
    //   System.out.println("i value: " + Integer.toString(i) + ". Value:  " + id[i]);
    // }

  
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
