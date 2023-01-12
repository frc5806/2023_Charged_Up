package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.kIntakeMotorPort, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // pass for now
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    
}
