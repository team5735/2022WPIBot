package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BrakeCommand extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    public BrakeCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SwerveModuleState frontLeftState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(-45));
        SwerveModuleState frontRightState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(45));
        SwerveModuleState backLeftState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(45));
        SwerveModuleState backRightState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(-45));
        SwerveModuleState[] moduleStates = { frontLeftState, frontRightState, backLeftState, backRightState };
        this.swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModuleState frontLeftState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(0));
        SwerveModuleState frontRightState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(0));
        SwerveModuleState backLeftState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(0));
        SwerveModuleState backRightState = new SwerveModuleState(0.001, Rotation2d.fromDegrees(0));
        SwerveModuleState[] moduleStates = { frontLeftState, frontRightState, backLeftState, backRightState };
        this.swerveSubsystem.setModuleStates(moduleStates);
    }
}