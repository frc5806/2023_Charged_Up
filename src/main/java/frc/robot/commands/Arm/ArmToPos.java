package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm.Arm;

public class ArmToPos extends PIDCommand {    
    public ArmToPos(double pos, Arm arm) {
        super(
            new PIDController(0.1, 0, 0),
            // Close loop on encoder position
            arm::getEncoderExtendPos,
            // Set reference to target
            pos,
            // Pipe output to move arm
            output -> arm.extend(output),
            // Require the arm
            arm);
    
      
        getController()
            .setTolerance(1, 10);
      }
    
      @Override
      public boolean isFinished() {
        // End when the controller is at the reference.
        return getController().atSetpoint();
      }
}
