package frc.robot.subsystems;

import java.math.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ClawConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax armMotor;
    CANSparkMax armMotorExtention;
    private final RelativeEncoder encoder;
    private final RelativeEncoder encoder2;
    double armRotatePos;
    double armExtentionPos;

    public Arm() {
        armMotor = new CANSparkMax(ClawConstants.kClawMotorPort, MotorType.kBrushless); 
        armMotorExtention = new CANSparkMax(ClawConstants.kClawMotorPort, MotorType.kBrushless);

        encoder = armMotor.getEncoder();
        encoder2 = armMotorExtention.getEncoder();

        armRotatePos = getEncoderRotatePos();
        armExtentionPos = getEncoderExtendPos();
        resetEncoders();
    }

    public void rotate(double power){
        armMotor.set(power);
        armRotatePos = getEncoderRotatePos();

    }

    public void extend(double power){
        armMotorExtention.set(power);
        armExtentionPos = getEncoderExtendPos();
    }


    public double getEncoderRotatePos() {
        return encoder.getPosition();
    }

    public double getEncoderExtendPos() {
        return encoder2.getPosition();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
        encoder2.setPosition(0);
    }
    
}
