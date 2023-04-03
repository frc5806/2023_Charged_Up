package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ArmExtension extends PIDSubsystem {
    CANSparkMax armMotorExtention;
    private final RelativeEncoder encoder;
    double armExtentionPos;

    public ArmExtension() {
        super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
        getController().setTolerance(20);
        setSetpoint(10);


        armMotorExtention = new CANSparkMax(ArmConstants.kArmMotorPort1, MotorType.kBrushless);
        encoder = armMotorExtention.getEncoder();

        armExtentionPos = getEncoderExtendPos();

        resetEncoders();
    }

    public void setSetpointArm(double setpoint){
        setSetpoint(setpoint);
    }


    @Override
    public void useOutput(double output, double setpoint) {
      armMotorExtention.set(output + getController().calculate(getMeasurement(), setpoint));
    }
  
    @Override
    public double getMeasurement() {
      return encoder.getPosition();
    }
  
    public boolean atSetpoint() {
      return m_controller.atSetpoint();
    }

    
    public void extend(double power){
        armMotorExtention.set(power);
        armExtentionPos = getEncoderExtendPos();
    }


    public double getEncoderExtendPos() {
        return encoder.getPosition();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

}
