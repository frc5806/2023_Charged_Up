package frc.robot.subsystems;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Arm.Arm;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    private final DifferentialDrive drivetrain;
    private final DifferentialDriveOdometry odometry;

    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;

    private final boolean isFirstPath = true;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private final AnalogInput input;
    private final AnalogPotentiometer distanceSensor;
    private final AnalogInput input1;
    private final AnalogPotentiometer distanceSensor1;
    
    private final Timer timer;

    DifferentialDriveKinematics kinematics;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);

  /** Creates a new ExampleSubsystem. */
    public DriveTrain() {
        rightMotors = new CANSparkMax[] {
            new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless),
            new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless),
            new CANSparkMax(DriveConstants.kRightMotor3Port, MotorType.kBrushless)
        };

        leftMotors = new CANSparkMax[] {
            new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless),
            new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless),
            new CANSparkMax(DriveConstants.kLeftMotor3Port, MotorType.kBrushless)
        };

        drivetrain = new DifferentialDrive(
            new MotorControllerGroup(rightMotors),
            new MotorControllerGroup(leftMotors));

        motorConfig();

       // rightEncoder = rightMotors[0].getAbsoluteEncoder(kDutyCycle);
      //  leftEncoder = leftMotors[0].getAbsoluteEncoder(null);

        rightEncoder = rightMotors[0].getEncoder();
        leftEncoder = leftMotors[0].getEncoder();

        kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

        // fix left and right distance
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);


        input = new AnalogInput(DriveConstants.kUltrasonicPort);
        input1 = new AnalogInput(DriveConstants.kUltrasonicPort1);

        input.setAverageBits(2);
        input1.setAverageBits(2);

        distanceSensor = new AnalogPotentiometer(input, 500, -30);
        distanceSensor1 = new AnalogPotentiometer(input1, 500, -30);

        timer = new Timer();
    }

    public void motorConfig(){
        for (CANSparkMax motor: leftMotors){
            motor.setInverted(false);
        }

        for (CANSparkMax motor: rightMotors){
            motor.setInverted(true);
        }
    }

// DriveTrain Functions
    public void stop() {
        drivetrain.stopMotor();
    }

    public void arcadeDrive(double speed, double turn, boolean squareInputs){
        drivetrain.arcadeDrive(speed, turn, squareInputs);
    }
    public void drive(double speed, double turn, boolean inPlace){
        drivetrain.curvatureDrive(speed, turn, inPlace);
    }
    public void safteyDrive(){
        arcadeDrive(0, 0, true);
    }

// Encoder

    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }

    // public void resetEncoders() {
    //     leftEncoder.reset();
    //     rightEncoder.getInverted()
    // }

    public void resetOdometry(Pose2d pose) {
      //  resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
      }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
      }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        for (CANSparkMax leftMotor : leftMotors){
            leftMotor.setVoltage(leftVolts);
        }
        for (CANSparkMax rightMotor : rightMotors) {
            rightMotor.setVoltage(rightVolts);
        }
        drivetrain.feed();
      }

    // Velocity

    public double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }

// Distance Sensor

    public double getUltrasonicDistance(){
       return distanceSensor.get();
    }

    public double getUltrasonicDistance1(){
        return distanceSensor1.get();
    }

// Gyro 
    public double getAngle() {
        return gyro.getRotation2d().getDegrees();
    }

    public Rotation2d geRotation2d(){
        return gyro.getRotation2d();
    }

    public void resetAngle(){
        gyro.reset();
    }

    public double getTurnRate(){
        return gyro.getRate();
    }

    public void setMaxOutput(double max) {
        drivetrain.setMaxOutput(max);
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    // public static void autoBalance() {

    //     in

    // }


    // public Command followTrajectoryCommand(PathWeaverTrajectory traj, boolean isFirstPath) {

    // }
    ///////////////////////////////////////////////////////////////////////////////
    public void stupidAutoDriveForwardOutake0(DriveTrain driveTrain, Intake intake, double speed) {
        // run intake

        
        while (Timer.getFPGATimestamp() < 2.5) {
            intake.runIntake(0.8);
        } // while loop not necessary but whatever.

        // drive backwards for 2.5 seconds
        while (Timer.getFPGATimestamp() > 2.5  && Timer.getFPGATimestamp() < 5) {
            driveTrain.arcadeDrive(-speed, 0, false);
        }
        
        // stop
        while (Timer.getFPGATimestamp() > 5) {
            driveTrain.safteyDrive();
        }
    }


    public void stupidAutoDriveForwardOutake1(DriveTrain driveTrain, Intake intake, double speed) {

        System.out.println("stupidAutoDriveOutake 1 started");
        // double currentEncoderTicksBackUp = driveTrain.getDistance();
        // double backUpDistance = currentEncoderTicksBackUp * DriveConstants.kDriveTickToFeet;
        
        // driveTrain.safteyDrive();

        // Forward intake
        timer.reset();
        while (timer.getFPGATimestamp() < 7) {
            // intake.runIntake(0.8);
 
            intake.setintakePos(speed);
            System.out.println("Intake running");
        }

        // intake.runIntake(0.8);




        // while ((backUpDistance < AutoConstants.backUpFeet)) {
        //     System.out.print("Back up distance: ");
        //     System.out.println(backUpDistance);
        //     driveTrain.arcadeDrive(-speed, 0, false);

        //     currentEncoderTicksBackUp = driveTrain.getDistance();
        //     backUpDistance = currentEncoderTicksBackUp * DriveConstants.kDriveTickToFeet;

        //     System.out.println("back up distance condition loop end");

        // }


            System.out.println(timer.getFPGATimestamp());
            // Timer.getFPGATimestamp() > 7 &&  Timer.getFPGATimestamp() < 11
        while (timer.getFPGATimestamp() > 7 &&  timer.getFPGATimestamp() < 11) {
            driveTrain.arcadeDrive(-speed,0,false);
            System.out.println(Timer.getFPGATimestamp());
            System.out.println("drive");


        }

        // Timer.delay(3);
        // driveTrain.arcadeDrive(-speed,0,false);


        // while (Timer.getFPGATimestamp() )

        driveTrain.safteyDrive();

        System.out.println("End of autonomous stupidAutoDriveForwardOutake1");
    }

    // This runs stupidAuto #1
    public Command stupidAutoDriveForwardOutake1Command(DriveTrain driveTrain, Intake intake, double speed){
        return this.runOnce(() -> this.stupidAutoDriveForwardOutake1(driveTrain, intake, speed));
    }

    public void stupidAutoArmBackwards(DriveTrain driveTrain, Intake intake, double speed) {
        // rotate the arm
        


        // release the game piece


        // drive backwards


        // then stop


    }

    public Command stupidAutoArmBackwards(DriveTrain driveTrain, Intake intake, Double speed) {
        return this.runOnce(() -> this.stupidAutoArmBackwards(driveTrain, intake, speed));
    }

    // make a command that just drives backwards for a certain amount of time then stops
    public void stupidAutoDriveBackwards(DriveTrain driveTrain, Double speed) {

        // // THIS IS STUPID. THE TIME WILL CHANGE EVERY TIME THE FUNCTION IS RAN SO IT WON'T ACTUALLY SAVE THE START TIME
        // // ADD ON AUTO INIT OR JUST USE TIME 0
        // double startTime = Timer.getFPGATimestamp();

        // find time difference 2.5 secs
        if (Timer.getFPGATimestamp()  < 2.5) {
            driveTrain.arcadeDrive(-speed, 0, false);
        } else {
            driveTrain.safteyDrive();
        }

        // while (Timer.getFPGATimestamp() > 2.5  && Timer.getFPGATimestamp() < 5) {
        //     driveTrain.arcadeDrive(-speed, 0, false);
        // } 

        // if (Timer.getFPGATimestamp() > 5) {
        //     driveTrain.safteyDrive();
        // }


        // driveTrain.safteyDrive();

    }

    public Command stupidAutoDriveBackwardsCommand(DriveTrain driveTrain, Double speed) {
        return this.runOnce(() -> this.stupidAutoDriveBackwards(driveTrain, speed));
    }







    ////////////////////////////////////////////////////////////////////////////////

    @Override
    public void periodic() {
        odometry.update(
            gyro.getRotation2d(), getLeftDistance(), getRightDistance());
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
