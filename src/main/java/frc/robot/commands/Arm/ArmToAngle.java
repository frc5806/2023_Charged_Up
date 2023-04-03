package frc.robot.commands.Arm;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm.Arm;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class ArmToAngle extends PIDCommand {
  Arm arm;
  public ArmToAngle(double angle, Arm arm) {
    super(
        new PIDController(0.1, 0, 0),
        arm::getAngle,
        angle,
        output -> arm.rotate(output),
        arm);

    this.arm = arm;
    getController().enableContinuousInput(-180, 180);
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  public double getError(){
    return getController().getPositionError();
  }

  public double getPeriod(){
    return getController().getPeriod();
  }

  public double getAngle(){
    return arm.getAngle();
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
