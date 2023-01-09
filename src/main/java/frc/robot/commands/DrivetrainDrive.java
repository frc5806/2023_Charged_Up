package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class DrivetrainDrive extends CommandBase {
    public final DriveTrain drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotation;

    public DrivetrainDrive(DriveTrain drivetrain, DoubleSupplier forward, DoubleSupplier rotation){
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.rotation = rotation;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        drivetrain.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble(), false);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.safteyDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
