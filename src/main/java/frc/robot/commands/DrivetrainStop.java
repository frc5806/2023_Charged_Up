package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainStop extends CommandBase {
    public final DriveTrain drivetrain;

    public DrivetrainStop(DriveTrain drivetrain){
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
