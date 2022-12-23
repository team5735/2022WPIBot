// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.trajectories.Trajectories;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final XboxController driverController, subsystemController;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.driverController = new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    this.subsystemController = new XboxController(Constants.OIConstants.SUBSYSTEM_CONTROLLER_PORT);

    this.swerveSubsystem = new SwerveSubsystem();

    // Configure the button bindings
    this.configureDriverBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverBindings() {
    // When there's no other command scheduled, drive the robot with Xbox joysticks
    this.swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(this.swerveSubsystem,
        () -> this.driverController.getLeftX(),
        () -> this.driverController.getLeftY(),
        () -> this.driverController.getRightX()));

    // When Y is pressed on driver controller, toggle field oriented
    new JoystickButton(this.driverController, XboxController.Button.kY.value)
        .whenPressed(new InstantCommand(() -> {
          this.swerveSubsystem.toggleFieldOriented();
        }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The trajectory to follow
    Trajectory trajectory = Trajectories.TEST_TRAJECTORY;

    PIDController xController = new PIDController(Constants.AutoConstants.AUTO_XCONTROLLER_KP, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.AUTO_YCONTROLLER_KP, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.AUTO_THETACONTROLLER_KP, 0, 0,
        Constants.AutoConstants.AUTO_THETACONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, // The trajectory to follow
        swerveSubsystem::getPose, // The supplier of the robot's x/y position and heading
        Constants.DrivetrainConstants.DT_KINEMATICS, // The kinematics of the robot
        xController, // The PID controller to correct error in the robot's x position
        yController, // The PID controller to correct error in the robot's y position
        thetaController, // The PID controller to correct error in the robot's heading
        swerveSubsystem::setModuleStates, // The function to use to set the robot's module states
        swerveSubsystem); // The subsystem to execute the command on

    return new SequentialCommandGroup(
        // Start the command by "placing" the robot at the beginning of the trajectory
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        // Run the trajectory command
        swerveControllerCommand,
        // Stop the robot at the end of the trajectory
        new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
