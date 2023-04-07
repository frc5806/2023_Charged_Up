package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    private final CANSparkMax intakeMotor3;
    // private final CANSparkMax intakeMotor4;

    private final RelativeEncoder encoder;
    
    public Intake() {
        intakeMotor1 = new CANSparkMax(IntakeConstants.kIntakeMotorPort1, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(IntakeConstants.kIntakeMotorPort2, MotorType.kBrushless);
        intakeMotor3 = new CANSparkMax(IntakeConstants.kIntakeMotorPort3, MotorType.kBrushless);
        // intakeMotor4 = new CANSparkMax(IntakeConstants.kIntakeMotorPort4, MotorType.kBrushless);

        encoder = intakeMotor2.getEncoder();
        resetEncoder();
    }

    public Command runIntake(double pwr) {
        return this.startEnd(() -> this.set(pwr), () -> this.set(0));
    }

    public void set(double pwr ) {
         intakeMotor1.set(pwr);
    }

    public void setintakePos(double pwr){
        intakeMotor2.set(pwr);
    }

    public Command adjustAngle(double pwr) {
        return this.startEnd(() -> this.setintakePos(pwr), () -> this.setintakePos(0));
    }

    public void winch(double pwr){
        intakeMotor3.set(pwr);
    }

    public Command winchIntake(double pwr) {
        return this.startEnd(() -> this.winch(pwr), () -> this.winch(0));
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
    
    public static class OurPneumatics {
        private Solenoid pistonIntake1;
        private Solenoid pistonIntake2;
        
        public boolean intakeEnabled = false;

        private static Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

        public OurPneumatics() {
            pistonIntake1 = new Solenoid(0, PneumaticsModuleType.CTREPCM, PneumaticsConstants.kPneumaticsPortL);
            pistonIntake2 = new Solenoid(0, PneumaticsModuleType.CTREPCM, PneumaticsConstants.kPneumaticsPortR);
        }

        public void changeIntakeMode() {
            intakeEnabled = !intakeEnabled;
            pistonIntake1.set(intakeEnabled);
            pistonIntake2.set(intakeEnabled);
        }

        public Command reverseIntake() {
            return Commands.runOnce(() -> changeIntakeMode());
        }

        public static void enableCompressor() {
            compressor.enableHybrid(40,60); 

            // Change from the factory preset to 60 psi on the pressure switch 
            // 
            // boolean tankFull = !compressor.getPressureSwitchValue();

            // compressor.enableDigital();
            // if (tankFull) {
            //     compressor.disable();
            // }
            // compressor.getCurrent();
        }
    }

}