package frc.robot.commands.Claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

public class ClawOpen extends PIDCommand {
    
    public ClawOpen(Claw claw) {
        super(
            new PIDController(0.4, 0.02, 0.02),
            // Close loop on encoder position
            claw::getEncoderPosition,
            // Set reference to target
            ClawConstants.clawPosOpen,
            // Pipe output to move claw
            output -> claw.set(output),
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
