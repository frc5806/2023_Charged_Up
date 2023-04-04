package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    // private final CANSparkMax intakeMotor3;
    // private final CANSparkMax intakeMotor4;

    private final RelativeEncoder encoder;
    
    public Intake() {
        intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotorPort1, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotorPort2, MotorType.kBrushless);
        // intakeMotor3 = new CANSparkMax(IntakeConstants.kIntakeMotorPort3, MotorType.kBrushless);
        // intakeMotor4 = new CANSparkMax(IntakeConstants.kIntakeMotorPort4, MotorType.kBrushless);

        encoder = intakeMotor2.getEncoder();
        resetEncoder();
    }

    public Command runIntake(double pwr) {
        return this.startEnd(() -> this.set(pwr), () -> this.set(0));
    }

    public void set(double pwr ) {
         intakeMotor1.set(pwr);
        // intakeMotor2.set(pwr);

        // intakeMotor3.set(pwr);
        // intakeMotor4.set(pwr);
    }

    public void setintakePos(double pwr){
        intakeMotor2.set(pwr);
    }

    public double getIntakeEncoderPos(){
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
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
