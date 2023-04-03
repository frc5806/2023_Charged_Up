package frc.robot.commands.Claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class ClawClose extends PIDCommand {
    
    public ClawClose(Claw claw) {
        super(
            new PIDController(0.2, 0.02, 0.04),
            // Close loop on encoder position
            claw::getEncoderPosition,
            // Set reference to target
            ClawConstants.clawPosClose,
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
