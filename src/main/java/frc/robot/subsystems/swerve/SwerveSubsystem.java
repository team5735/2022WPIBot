package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*
    Robot Reference Frame
  
    +Rotation: Clockwise (Rotate Left)

             +X 
         ___________
        |     F     |
        |           |
   +Y   |L         R|    -Y
        |           |
        |_____B_____|

             -X
 */

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometer;
    private final Field2d m_field = new Field2d();

    private boolean isFieldOrientedEnabled = false;

    public SwerveSubsystem() {

        this.frontLeft = new SwerveModule(
                Constants.MotorConstants.FRONT_LEFT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.FRONT_LEFT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.FRONT_LEFT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS,
                1,
                Constants.MotorConstants.FRONT_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.FRONT_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.frontRight = new SwerveModule(
                Constants.MotorConstants.FRONT_RIGHT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.FRONT_RIGHT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.FRONT_RIGHT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS,
                2,
                Constants.MotorConstants.FRONT_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.FRONT_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.backLeft = new SwerveModule(
                Constants.MotorConstants.BACK_LEFT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.BACK_LEFT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.BACK_LEFT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS,
                3,
                Constants.MotorConstants.BACK_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.BACK_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.backRight = new SwerveModule(
                Constants.MotorConstants.BACK_RIGHT_DRIVE_MOTOR_ID,
                Constants.MotorConstants.BACK_RIGHT_TURN_MOTOR_ID,
                false, // drive motor reversed
                false, // turn motor reversed
                Constants.MotorConstants.BACK_RIGHT_ABS_ENCODER_CHANNEL,
                Constants.MotorConstants.BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS,
                4,
                Constants.MotorConstants.BACK_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS,
                Constants.MotorConstants.BACK_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS);

        this.gyro = new AHRS(SerialPort.Port.kUSB);
        this.odometer = new SwerveDriveOdometry(
                Constants.DrivetrainConstants.DT_KINEMATICS, // Give the odometry the kinematics of the robot,
                new Rotation2d(0)); // and the starting angle of the robot

        // Reset the heading of the robot after 1 second
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                this.zeroHeading();
            } catch (Exception e) {
                throw new RuntimeException("Could not reset the heading of the robot!");
            }
        }).start();

        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Toggles the field-oriented drive mode on and off
     */
    public void toggleFieldOriented() {
        this.isFieldOrientedEnabled = !this.isFieldOrientedEnabled;
    }

    /**
     * Returns if field-oriented drive is enabled
     * @return true if field-oriented drive is enabled, false otherwise
     */
    public boolean isFieldOrientedEnabled() {
        return this.isFieldOrientedEnabled;
    }

    /**
     * Resets the heading of the robot.
     */
    public void zeroHeading() {
        System.out.println("Resetting gyro heading to 0");
        this.gyro.reset();
    }

    /**
     * Gets the current heading of the gyro, in degrees
     * 
     * @return
     */
    public double getGyroHeadingDeg() {
        // Negative because NavX says clockwise is positive but WPILib says counterclockwise is positive
        return Math.IEEEremainder(-this.gyro.getAngle(), 360);
    }

    /**
     * Constructs a Rotation2d object from the gyro heading, converting it to
     * radians
     * 
     * @return the Rotation2d object
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getGyroHeadingDeg());
    }

    /**
     * Gets the current position of the robot on the field
     * 
     * @return the current position of the robot
     */
    public Pose2d getPose() {
        return this.odometer.getPoseMeters();
    }

    /**
     * Reset the position and heading of the robot to a known location and
     * orientation
     * 
     * @param pose the new position and orientation of the robot
     */
    public void resetOdometry(Pose2d pose) {
        this.odometer.resetPosition(pose, getRotation2d());
    }

    /**
     * Sets the desired velocity and angle of all swerve modules
     * 
     * @param desiredStates
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Scales all desired states to the maximum speed of the robot
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * The subsystem method called every 20ms. Updates the odometry of the robot.
     */
    @Override
    public void periodic() {
        odometer.update( // Updates the current angle and position of the robot
                getRotation2d(),
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState());

        m_field.setRobotPose(this.odometer.getPoseMeters());

        SmartDashboard.putBoolean("Gyro Calibrating", this.gyro.isCalibrating());
        SmartDashboard.putBoolean("Gyro Connected", this.gyro.isConnected());
        SmartDashboard.putNumber("Robot Heading (deg)", this.getGyroHeadingDeg());
        SmartDashboard.putString("Robot Location (m)", this.getPose().getTranslation().toString());
        SmartDashboard.putBoolean("Field Oriented Enabled", this.isFieldOrientedEnabled);
    }

    /**
     * Stops all swerve modules
     */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}
