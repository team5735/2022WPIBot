// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import cfutil.CharacterizationConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

        public static class SpeedConstants {
                // TODO: Be aware these numbers were arbitrarily set
                public static final double MAX_SPEED_METERS_PER_SECOND = 3.0; // 3 meters per second
                public static final double MAX_ACCELERATION_METERS_PER_SECONDSQ = 3.0; // 3 meters per second squared,
                                                                                       // so 1
                                                                                       // second to full speed
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second
                public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ = Math.PI; // 1/2 rotation per
                                                                                                    // second
                                                                                                    // squared, so 1
                                                                                                    // second to
                                                                                                    // full speed
        }

        public static class DrivetrainConstants {
                // TODO: Double check these numbers
                public static final double WHEEL_RADIUS = 0.0508; // meters
                public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // meters
                public static final double MK3_DRIVE_MOTOR_GEAR_RATIO = (48. / 16.) * (16. / 28.) * (60. / 15.); // (48*16*60)/(16*28*15)
                                                                                                                 // =
                                                                                                                 // 46080
                                                                                                                 // /
                                                                                                                 // 6720
                                                                                                                 // =
                                                                                                                 // 48:7
                                                                                                                 // =
                                                                                                                 // 6.85714286
                public static final double MK3_TURN_MOTOR_GEAR_RATIO = (32. / 15.) * (60. / 15.); // 128:15 = 8.5333

                // TODO: Measure track width and wheel base

                // Locations of swerve modules, in meters, from the center of the robot
                // Distance between right and left wheels, in m
                public static final double DT_TRACK_WIDTH = Units.inchesToMeters(21);
                // Distance between front and back wheels, in m
                public static final double DT_WHEEL_BASE = Units.inchesToMeters(25.5);

                public static final Translation2d locationFL = new Translation2d(DT_WHEEL_BASE / 2, DT_TRACK_WIDTH / 2);
                public static final Translation2d locationFR = new Translation2d(DT_WHEEL_BASE / 2,
                                -DT_TRACK_WIDTH / 2);
                public static final Translation2d locationBL = new Translation2d(-DT_WHEEL_BASE / 2,
                                DT_TRACK_WIDTH / 2);
                public static final Translation2d locationBR = new Translation2d(-DT_WHEEL_BASE / 2,
                                -DT_TRACK_WIDTH / 2);

                public static final SwerveDriveKinematics DT_KINEMATICS = new SwerveDriveKinematics(locationFL,
                                locationFR, locationBL, locationBR);
        }

        public static class MotorConstants {
                public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
                public static final int FRONT_LEFT_TURN_MOTOR_ID = 2;
                public static final int FRONT_LEFT_ABS_ENCODER_CHANNEL = 1;
                public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = 0.679;

                public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 3;
                public static final int FRONT_RIGHT_TURN_MOTOR_ID = 4;
                public static final int FRONT_RIGHT_ABS_ENCODER_CHANNEL = 2;
                public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = 0.388;

                public static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
                public static final int BACK_LEFT_TURN_MOTOR_ID = 6;
                public static final int BACK_LEFT_ABS_ENCODER_CHANNEL = 3;
                public static final double BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = 0.117;

                public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 7;
                public static final int BACK_RIGHT_TURN_MOTOR_ID = 8;
                public static final int BACK_RIGHT_ABS_ENCODER_CHANNEL = 4;
                public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = 0.549;

                // TODO: Populate characterization constants
                public static final CharacterizationConstants DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.0, 0.0, 0.0)
                                .setFeedbackConstants(0.0, 0.0, 0.0)
                                .build();

                public static final CharacterizationConstants TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.0, 0.0, 0.0)
                                .setFeedbackConstants(0.0, 0.0, 0.0)
                                .build();
        }

        public static final class AutoConstants {
                public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND
                                / 4; // During auto, 1/4 of max speed
                public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
                                / 10; // During auto, 1/10 of max angular speed
                public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECONDSQ = Constants.SpeedConstants.MAX_ACCELERATION_METERS_PER_SECONDSQ
                                / 2; // During auto, 1/2 of max acceleration
                public static final double AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ = Constants.SpeedConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ
                                / 4; // During auto, 1/4 of max angular acceleration

                // TODO: Tune these auto controller constants
                public static final double AUTO_XCONTROLLER_KP = 1.5;
                public static final double AUTO_YCONTROLLER_KP = 1.5;
                public static final double AUTO_THETACONTROLLER_KP = 3;

                public static final TrapezoidProfile.Constraints AUTO_THETACONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
                                AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ);

                public static final TrajectoryConfig AUTO_TRAJECTORY_CONFIG = new TrajectoryConfig(
                                AUTO_MAX_SPEED_METERS_PER_SECOND,
                                AUTO_MAX_ACCELERATION_METERS_PER_SECONDSQ)
                                .setKinematics(DrivetrainConstants.DT_KINEMATICS);
        }

        public static class OIConstants {
                public static final int DRIVER_CONTROLLER_PORT = 0;
                public static final int SUBSYSTEM_CONTROLLER_PORT = 1;
                public static final double JOYSTICK_DEADBAND = 0.05;
        }

}
