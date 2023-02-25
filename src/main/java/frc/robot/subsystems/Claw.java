package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ClawConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    CANSparkMax clawMotor;
    private final RelativeEncoder encoder;

    public Claw() {
        clawMotor = new CANSparkMax(ClawConstants.kClawMotorPort, MotorType.kBrushless);
        encoder = clawMotor.getEncoder();
        resetEncoder();
    }

    public void setPower(double power){
        clawMotor.set(power);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }
    
}
