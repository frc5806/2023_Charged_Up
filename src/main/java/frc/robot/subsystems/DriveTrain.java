package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

    private final DifferentialDrive drivetrain;
    private final DifferentialDriveOdometry odometry;

    private final Encoder rightEncoder;
    private final Encoder leftEncoder;

    private final boolean isFirstPath = true;

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    DifferentialDriveKinematics kinematics;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0, 1.0);

    int P, I, D = 1;
    int integral, previous_error, setpoint = 0;
    double rcw;

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

        rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1]); 
        leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1]);

        leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();

        kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);

        // fix left and right distance
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
    }

// config
    public void motorConfig() {
        for (CANSparkMax motor : leftMotors){
            motor.setInverted(false);
        }

        for (CANSparkMax motor : rightMotors){
            motor.setInverted(true);
        }
    }

    public void setSetpoint(int setpoint)
    {
        this.setpoint = setpoint;
    }

    public void PID(){
        double error = setpoint - gyro.getAngle(); // Error = Target - Actual
        this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        double derivative = (error - this.previous_error) / .02;
        this.rcw = P*error + I*this.integral + D*derivative;
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
        return leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return rightEncoder.getDistance();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance())/2;
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
      }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
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
        return leftEncoder.getRate();
    }

    public double getRightVelocity() {
        return rightEncoder.getRate();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity())/2;
    }

// Gyro 
    public double getAngle() {
        return gyro.getRotation2d().getDegrees();
    }

    public void resetAngle(){
        gyro.reset();
    }

    public double getTurnRate(){
        return gyro.getRate();
    }

    public double getKinematics() {
        // TODO: Update
        return 0;
    }

    public void setMaxOutput(double max) {
        drivetrain.setMaxOutput(max);
    }

     public double outtakeVolts(){
         return 0;
     }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }


    // public Command followTrajectoryCommand(PathWeaverTrajectory traj, boolean isFirstPath) {

    // }

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
