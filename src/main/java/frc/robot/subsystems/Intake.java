package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    public Intake() {
        intakeMotor = new CANSparkMax(Constants.kIntakeMotorPort, MotorType.kBrushless);
    }

    public Command runIntake(double pwr) {
        return this.startEnd(() -> this.set(pwr), () -> this.set(0.0));
    }

    public void set(double pwr ) {
        intakeMotor.set(pwr);
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
