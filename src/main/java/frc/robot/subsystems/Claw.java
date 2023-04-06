/* Class creates claw: Assigns claw motors and encoders, controls the powers, and resets/gets encoder position. */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    CANSparkMax clawMotor;
    private final RelativeEncoder encoder;

    public Claw() {
        clawMotor = new CANSparkMax(ClawConstants.kClawMotorPort, MotorType.kBrushless);
        encoder = clawMotor.getEncoder();
        resetEncoder();
    }

    public void set(double power){
        clawMotor.set(power);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public Command runClaw(double pwr) {
        return this.startEnd(() -> this.set(pwr), () -> this.set(0));
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }
    
}
