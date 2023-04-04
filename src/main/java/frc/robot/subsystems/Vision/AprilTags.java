package frc.robot.subsystems.Vision;

// package com.stuypulse.stuylib.network.limelight;
// import edu.wpi.first.apriltag.AprilTag;

import edu.wpi.first.apriltag.AprilTag;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.limelight.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Field.FieldConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;




public class AprilTags extends SubsystemBase {

    private final NetworkTable networkTable =  NetworkTableInstance.getDefault().getTable("limelight");
    
    double[] cropValues = new double[4];

    private Map<Integer, Double> lastTagDetection = new HashMap<>();

    List<Pose3d> tagPoses = new ArrayList<>();

    // Pose3d cameraPose3d = new Pose3d();

    /* Rotation set to (0, 0, 0) -> Camera facing forward
        X -> forward/backward from center
        Y -> left/right from center
        Z -> up/down from center (height) */
    Transform3d robotToCamLocation = new Transform3d(
        new Translation3d(
        VisionConstants.kvisionCameraX, 
        VisionConstants.kvisionCameraY, 
        VisionConstants.kvisionCameraZ), 
    new Rotation3d(0,0,0));

    // THIS MIGHT NEED SOME FIXING.
    public static double[] getAprilID() {
        // return Limelight.table.getEntry("tid").getDoubleArray(new double[6]);
        return Limelight.table.getEntry("tid").getDoubleArray(new double[6]);
    }


    // Double getAprilID() {
    //     return networkTable.getEntry("tid").getDoubleArray(new double[6]);
    // }


    void cropValues() {
        cropValues[0] = -1.0;
        cropValues[1] = 1.0;
        cropValues[2] = -1.0;
        cropValues[3] = 1.0;
        // networkTable.getEntry("crop").setDoubleArray(cropValues);
        Limelight.table.getEntry("crop").setDoubleArray(cropValues);

    }

    public AprilTags() {
        FieldConstants.aprilTags.getTags().forEach(
            (AprilTag tag) -> {
                lastTagDetection.put(tag.ID, 0.0);
            });
    }

    // void updateLastDetection() {
    //     for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
    //         int tagId = (int) values[i];
    //         lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
    //         Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) values[i]);
    //         if (tagPose.isPresent()) {
    //           tagPoses.add(tagPose.get());
    //         }
    //       }
    // }

    // void updateLastDetection() {
    //     lastTagDetection.put()

    //     for (int i = 0; i < )
    // }


        public static class AprilAutonomous {
            /*
                TO-DO: 
                Set starting positions (by using Field Constants)
                Drive to closest tag (by using limelight data; find distance.)

                Useful to estimate Pose: https://docs.photonvision.org/en/latest/docs/examples/simposeest.html
            
                *** Also look at notepad notes.
            */


            public static void moveToApril() {
                // if april tag detected
                // Calculate distance from center of robot to april
                // move until april is half the center of robot away from april tag



            }
        }


}
