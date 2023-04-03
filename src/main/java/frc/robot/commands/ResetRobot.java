package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.commands.Arm.ArmToAngle;
import frc.robot.commands.Arm.ArmToPos;
import frc.robot.commands.Claw.ClawPos;

public class ResetRobot extends ParallelCommandGroup {
    public ResetRobot(RobotContainer robot) {
        // addCommands(new ArmToPos(0, robot.getArm()),
        //             new ArmToAngle(0, robot.getArm()),
        //             new ClawPos(ClawConstants.clawPosOpen, robot.getClaw()));
    }
}