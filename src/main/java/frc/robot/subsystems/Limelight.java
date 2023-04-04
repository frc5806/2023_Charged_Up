package frc.robot.subsystems;

import java.lang.Math;
import java.text.spi.DecimalFormatSymbolsProvider;

// package com.stuypulse.stuylib.network.limelight;


import com.kauailabs.navx.IMUProtocol;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.stuylib.network.limelight.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveTrain.TurnToAngle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Limelight extends SubsystemBase {
    
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");    
    static NetworkTableEntry ta = table.getEntry("ta");
    static NetworkTableEntry tv = table.getEntry("tv");

    public static double x;
    public static double y;
    public static double area;
    public static boolean validTargets; 

    public static class LimelightData {

        /*---- Functions ----- */
        public static void update() {
            //read values periodically
            x = tx.getDouble(0.0);
            y = ty.getDouble(0.0);
            area = ta.getDouble(0.0);
            validTargets = tv.getBoolean(false);
    
        }

        public static double getX() {
            return x;
        }
    
        public static double getY() {
            return y;
        }
    
        public static double getArea() {
            return area;
        }

        public static boolean isValidTargets() {
            return validTargets;
        }


        // Prints out on terminal x, y, z values from tables
        public static void dataTest() {

            LimelightData.update();
            double[] dataXYA = new double[]{LimelightData.getX(), LimelightData.getY(), LimelightData.getArea()};

            System.out.println("X: " + String.valueOf(dataXYA[0]) + ", Y: " + String.valueOf(dataXYA[1]) + ", Area: " + String.valueOf(dataXYA[2]));
            System.out.println("ValidTargets: " + Boolean.valueOf(validTargets));
        }

    }

    public static class LimelightMode {

        public static void driverCamera() {
            table.getEntry("camMode").setNumber(1);
            table.getEntry("ledMode").setNumber(1);
        }
    
        public static void retroCamera() {
            table.getEntry("camMode").setNumber(0);
            table.getEntry("ledMode").setNumber(3);
        }

        public static void setPipelineApril() {
            table.getEntry("pipeline").setNumber(VisionConstants.klimelightPipelineAprilTag);
        }

        public static void setPipelineVision() {
            table.getEntry("pipeline").setNumber(VisionConstants.klimelightPipelineVision);
        }
    }

    public static class PoseEstimators {

        static DifferentialDrivePoseEstimator poseEstimator;
        

        public PoseEstimators(DriveTrain driveTrain) {
            LimelightMode.setPipelineApril();

             DifferentialDrivePoseEstimator poseEstimator = // was final; might break code
                new DifferentialDrivePoseEstimator(
                    DriveConstants.kDriveKinematics, 
                    driveTrain.geRotation2d(), 
                    driveTrain.getLeftDistance(), 
                    driveTrain.getRightDistance(), 
                    new Pose2d(),
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        }

        public static void updatePoseEstimator(DriveTrain driveTrain) {

            poseEstimator.update(driveTrain.geRotation2d(), driveTrain.getLeftDistance(), driveTrain.getRightDistance());

            // relative robot pos to april tag

            // MAKE SURE TO CHANGE PIPELINE TO APRIL
            Pose3d visionMeasurement3d = 
                new Pose3d(new Translation3d(LimelightData.getX(), LimelightData.getY(), 0.0), new Rotation3d(0,0,0));
            

            Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();

            poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());



        }

        public static Pose2d getCurrentRobotPose() {
           Pose2d currentPose =  poseEstimator.getEstimatedPosition();
           return currentPose;
        }

        public static double getCurrentRobotPoseX() {
            return getCurrentRobotPose().getX();
        }

        public static double getCurrentRobotPoseY() {
            return getCurrentRobotPose().getY();
        }






        }
        
    
    public static class LimelightAutonomous {

        public static void limelightAutonomousDrive(DriveTrain driveTrain, double output) {
            // ***** FOR THIS METHOD, CHECK THAT Y IS ACTUALLY DIRECTION FACING TAG

            // get the current position
            // if the current distance (pose) form the tag is more than we want, drive until tag

            Pose2d currentPose = PoseEstimators.getCurrentRobotPose();
            

            double currentX = currentPose.getX();
            double currentY = currentPose.getY();

            // double startingTheta = Math.PI/2;
            double startingTheta = driveTrain.getAngle();
            double angleFromAprilTag = Math.atan(currentY/currentX);
            double angleToTurn = startingTheta - angleFromAprilTag; // pi/2 - tan^-1(y/x)

            // TURN ANGLE

            runTurnToAngleCommand(angleToTurn, driveTrain);

            // NOW MOVE FORWARD
            while (currentY - DriveConstants.kRobotLength > 0) {
                driveTrain.arcadeDrive(output, 0, false);

            }

            // NOW TURN AGAIN

            runTurnToAngleCommand(angleToTurn, driveTrain);


        }

        public static PIDCommand runTurnToAngleCommand(double angle, DriveTrain driveTrain) {
            return new TurnToAngle(angle, driveTrain);
        }

        
    }



    /*--------------------- Calculating distance-------------------------- */
    double targetOffsetAngle_Vertical = ty.getDouble(0.0); // vertical offset
    double targetOffsetAngle_Horizontal = tx.getDouble(0.0); // horzontal offset

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 25.0; // edit later

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0; // l

    // distance from the target to the floor
    double goalHeightInches = 41.0; // obtained from manual

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    double Kp = -0.1;
    double steering_adjust;

    public double getDistanceFromTarget(){
        return distanceFromLimelightToGoalInches;
    }

    
    
    /*---- Aiming---- */
    public void steerSideways(DriveTrain drivespark) {
    
    
        while (targetOffsetAngle_Horizontal >= 0 ) {
            double heading_error = tx.getDouble(0.0);
            steering_adjust = Kp * tx.getDouble(0.0);

            drivespark.arcadeDrive(0.0, steering_adjust, true);
            
        }
    }

    

    public void turnToApril(NetworkTable networkTable) {

        

    }




} 
