package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    /**
     * Creates a new SwerveJoystickCmd to drive the robot using an Xbox controller.
     * 
     * @param swerveSubsystem    the swerve subsystem
     * @param xSpdFunction       A percentage of the left joystick on the X axis
     * @param ySpdFunction       A percentage of the left joystick on the Y axis
     * @param turningSpdFunction A percentage of the right joystick on the X axis
     */
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.xLimiter = new SlewRateLimiter(Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND);
        this.yLimiter = new SlewRateLimiter(Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND);
        this.turningLimiter = new SlewRateLimiter(Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get() * 0.5;
        double ySpeed = ySpdFunction.get() * 0.5; // xbox controller
        double turningSpeed = turningSpdFunction.get() * 0.75;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.JOYSTICK_DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.JOYSTICK_DEADBAND ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.OIConstants.JOYSTICK_DEADBAND ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND; // returns desired
                                                                                                    // velocity in m/s
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND; // returns desired
                                                                                                    // velocity in m/s
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND; // returns desired angular velocity in
                                                                                 // rad/s

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (this.swerveSubsystem.isFieldOrientedEnabled()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = Constants.DrivetrainConstants.DT_KINEMATICS
                .toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
