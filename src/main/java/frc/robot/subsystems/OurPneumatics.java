package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class OurPneumatics extends SubsystemBase{

    // private Solenoid pistonIntake1;
    // private Solenoid pistonIntake2;
    private DoubleSolenoid dublenoid;

    public static boolean intakeEnabled = false;

    
    private static Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    public OurPneumatics() {
        // pistonIntake1 = new Solenoid(0, PneumaticsModuleType.CTREPCM, PneumaticsConstants.kPneumaticsPortL);
        // pistonIntake2 = new Solenoid(0, PneumaticsModuleType.CTREPCM, PneumaticsConstants.kPneumaticsPortR);
        dublenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 1);
        dublenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void changeIntakeMode() {
        // intakeEnabled = !intakeEnabled;
        // pistonIntake1.set(intakeEnabled);
        // pistonIntake2.set(intakeEnabled);
        dublenoid.toggle();
        // if (intakeEnabled) {
        //     dublenoid.toggle();
        //     System.out.println("0000");
        // } else {

        //     System.out.println("hy5treds");
        // }
        // pistonIntake1.set(intakeEnabled);
        // pistonIntake2.set(!intakeEnabled);

            // test push
        System.out.println("ran----------------------------------------------");
    }

    public Command reverseIntake() {
        return Commands.runOnce(() -> changeIntakeMode());
        // return this.startEnd(() -> this.changeIntakeMode(), () -> System.out.println("helake"));
    }
///
    // public Command reverseIntake0(boolean enabled) {
    //     return this.runOnce(() -> this.set(enabled));
    // }

    // public void set() {
    //     intakeEnabled = !intakeEnabled;
    //     pistonIntake1.set(intakeEnabled);
    // }



    // private Command startEnd(DriveTrain driveTrain, Intake intake) {
    //     return this.startEnd(() -> changeIntakeMode(), () -> System.out.println("helake"));
    // }




    public static void enableCompressor() {
        compressor.enableDigital();
    
}
    
}
