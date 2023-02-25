package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Claw;

public class ClawPos extends PIDCommand {
    
    public ClawPos(double pos, Claw claw) {
        super(
            new PIDController(0.7, 0.2, 0),
            // Close loop on encoder position
            claw::getEncoderPosition,
            // Set reference to target
            pos,
            // Pipe output to move claw
            output -> claw.setPower(output),
            // Require the claw
            claw);
    
      
        getController()
            .setTolerance(1, 10);
      }
    
      @Override
      public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
      }
}
