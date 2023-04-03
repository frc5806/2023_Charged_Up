package frc.robot.subsystems;

// package com.stuypulse.stuylib.network.limelight;


import com.kauailabs.navx.IMUProtocol;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.stuylib.network.limelight.*;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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

        public static void setPipeline(int pipelineNum) {
            table.getEntry("pipeline").setNumber(pipelineNum);
        }
    }

    public static class PoseEstimators {
        

        public static void initiatePoseEstimator(DriveTrain driveTrain, Pose2d initialPoseMeters) {

            final DifferentialDrivePoseEstimator poseEstimator =
                new DifferentialDrivePoseEstimator(
                    DriveConstants.kDriveKinematics, 
                    driveTrain.getAngle(), 
                    driveTrain.getLeftDistance(), 
                    driveTrain.getRightDistance(), 
                    new Pose2d(),
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));


            poseEstimator.update(driveTrain.getAngle(), driveTrain.getLeftDistance(), driveTrain.getRightDistance());

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
