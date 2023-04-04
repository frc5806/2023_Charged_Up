package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight.PoseEstimators;

public class AprilAutonomous extends PIDCommand{

    Pose2d currentPose = PoseEstimators.getCurrentRobotPose();
    double currentY = currentPose.getY();


    public AprilAutonomous(DriveTrain driveTrain) {
        super (
            new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD), 
            //
            PoseEstimators.getCurrentRobotPoseY(), // get current position 
            // Target output should be distance from front of apriltag
            DriveConstants.kRobotLength, 
            output -> driveTrain.arcadeDrive(0, output, false),       
            driveTrain);
            
    }
    
}
