package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax armMotor;
    CANSparkMax armMotorExtention;
    CANSparkMax armMotor1;
    CANSparkMax armMotor2;
    private final RelativeEncoder encoder;
    private final RelativeEncoder encoder2;
    double armRotatePos;
    double armExtentionPos;
    double angle;

    public Arm() {
        armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless); 
        armMotor1 = new CANSparkMax(ArmConstants.kArmMotorPort1, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(ArmConstants.kArmMotorPort2, MotorType.kBrushless);
        armMotorExtention = new CANSparkMax(ArmConstants.kArmMotorPort3, MotorType.kBrushless);

        encoder = armMotor.getEncoder();
        encoder2 = armMotorExtention.getEncoder();

        armRotatePos = getEncoderRotatePos();
        armExtentionPos = getEncoderExtendPos();

        updateAngle();
        resetEncoders();
    }

    public void rotate(double power){
        armMotor.set(power);
        armMotor1.set(power);
        armMotor2.set(power);

        armRotatePos = getEncoderRotatePos();
        updateAngle();
    }

    public void extend(double power){
        armMotorExtention.set(power);
        armExtentionPos = getEncoderExtendPos();
        updateAngle();
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

    public void updateAngle() {
        this.angle = getEncoderRotatePos()/ArmConstants.countsPerRev * 360;
    }

    public double getAngle(){
        return this.angle;
    }
    
}
