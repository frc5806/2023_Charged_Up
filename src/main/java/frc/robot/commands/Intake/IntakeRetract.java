package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeRetract extends PIDCommand {
    
    public IntakeRetract(Intake intake) {
        super(
            new PIDController(0.2, 0.02, 0.04),
            // Close loop on encoder position
            intake::getIntakeEncoderPos,
            // Set reference to target
            IntakeConstants.intakePosClose,
            // Pipe output to move claw
            output -> intake.setintakePos(output),
            // Require the claw
            intake);
    
      
        getController()
            .setTolerance(1, 10);
      }
    
      @Override
      public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
      }
}
