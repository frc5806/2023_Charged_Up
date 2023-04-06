package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class StupidForwardAutonomous extends PIDCommand {

    // REMEMBER TO CHANGE THE OUTPUT FUNCTION TO STUPID AUTONOMOUS
    public StupidForwardAutonomous(DriveTrain drivetrain, Intake intake, double distance) {
        super(
            new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD),
            // Close loop on heading
            drivetrain::getDistance,
            // Set reference to target
            distance,
            // Pipe output to turn robot
             output -> drivetrain.arcadeDrive(output, 0, false),

            // YOU CAN CHOOSE 0 OR 1 FOR STUPID AUTONOMOUS
         //   output -> drivetrain.stupidAutoDriveForwardOutake0(drivetrain, intake, distance),
            // Require the drive
            drivetrain);
    
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController()
            .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

        getController()
        .setTolerance(1, 10);
        
    }
}
