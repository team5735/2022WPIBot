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
                public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0); // meters // MK4 Billet wheel has 4 inches outer diameter, so 2 inches radius
                public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS; // meters
                public static final double MK4_DRIVE_MOTOR_GEAR_RATIO = 8.14 / 1.0;
                public static final double MK4_TURN_MOTOR_GEAR_RATIO = 12.8 / 1.0;

                // Locations of swerve modules, in meters, from the center of bthe robot
                // Distance between right and left wheels, in m
                public static final double DT_TRACK_WIDTH = Units.inchesToMeters(19.5);
                // Distance between front and back wheels, in m
                public static final double DT_WHEEL_BASE = Units.inchesToMeters(21.5);

                public static final Translation2d locationFL = new Translation2d(DT_WHEEL_BASE / 2,
                                 DT_TRACK_WIDTH / 2);
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
                // Offset values must be positive!
                
                // Module 4
                public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 7;
                public static final int FRONT_LEFT_TURN_MOTOR_ID = 8;
                public static final int FRONT_LEFT_ABS_ENCODER_CHANNEL = 4;
                public static final double FRONT_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = 1.302;//2.794;
                public static final CharacterizationConstants FRONT_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.60709, 2.7646, 0.15464/*0.6765, 2.8661, 0.096105*/) //0.60206, 2.8155. 0.30703 -- 0.5405, 2.8575, 0.31968
                                .setFeedbackConstants(2.64, 0.0, 0.0) //3.6151, new tested value: 3.8261
                                .build();
                public static final CharacterizationConstants FRONT_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                        .setFeedforwardConstants(0.61235, 0.21322, 0.0050862)
                        .setFeedbackConstants(4.404, 0.0, 0.16072)
                        .build();

                // Module 3
                public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 5;
                public static final int FRONT_RIGHT_TURN_MOTOR_ID = 6;
                public static final int FRONT_RIGHT_ABS_ENCODER_CHANNEL = 3;
                public static final double FRONT_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = 2.362;//1.857;
                public static final CharacterizationConstants FRONT_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                                .setFeedforwardConstants(0.49831, 2.6069, 0.13338) // 0.49794, 2.777, 0.11579 -- 0.40416, 2.8677, 0.15439 -- 0.49278, 2.6965, 0.14252
                                .setFeedbackConstants(2.5147/*2.6455*/, 0.0, 0.0) // 2.3645 -- 2.8886
                                .build();
                public static final CharacterizationConstants FRONT_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                        .setFeedforwardConstants(0.63202, 0.21446, 0.0045286)
                        .setFeedbackConstants(4.2017, 0.0, 0.14116)
                        .build();

                // Module 2
                public static final int BACK_LEFT_DRIVE_MOTOR_ID = 3;
                public static final int BACK_LEFT_TURN_MOTOR_ID = 4;
                public static final int BACK_LEFT_ABS_ENCODER_CHANNEL = 2;
                public static final double BACK_LEFT_ABS_ENCODER_OFFSET_ROTATIONS = 2.632;//1.113;
                public static final CharacterizationConstants BACK_LEFT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                        .setFeedforwardConstants(0.60709, 2.7646, 0.15464) //0.60206, 2.8155. 0.30703
                        .setFeedbackConstants(2.64, 0.0, 0.0) //3.6151
                        .build();
                public static final CharacterizationConstants BACK_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                        .setFeedforwardConstants(0.65777, 0.21604, 0.0052368) // 0.75777
                        .setFeedbackConstants(4.4082, 0.0, 0.16538)
                        .build();

                // Module 1
                public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 1;
                public static final int BACK_RIGHT_TURN_MOTOR_ID = 2;
                public static final int BACK_RIGHT_ABS_ENCODER_CHANNEL = 1;
                public static final double BACK_RIGHT_ABS_ENCODER_OFFSET_ROTATIONS = 1.432;//0.928;
                public static final CharacterizationConstants BACK_RIGHT_DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                        .setFeedforwardConstants(0.49831, 2.6069, 0.13338/*0.59757, 2.7237, 0.63104*/) // 0.49794, 2.777, 0.11579
                        .setFeedbackConstants(2.5147, 0.0, 0.0) // 2.3645
                        .build();
                public static final CharacterizationConstants BACK_RIGHT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                        .setFeedforwardConstants(0.62974, 0.21458, 0.0044083)
                        .setFeedbackConstants(4.1585, 0.0, 0.13685)
                        .build();

                // public static final CharacterizationConstants DRIVE_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                //                 .setFeedforwardConstants(0.67366, 2.9808, 0.088989)
                //                 .setFeedbackConstants(1.8428, 0.0, 0.0)
                //                 .build();

                                // .setFeedforwardConstants(0.32, 1.51, 0.27)
                                // .setFeedbackConstants(0.05, 0.0, 0.0)
                                // .build();

                // TODO: These turn motor constants are right for one motor, check for all motors
                // public static final CharacterizationConstants FRONT_LEFT_TURN_MOTOR_CHARACTERIZATION_CONSTANTS = new CharacterizationConstants.Builder()
                //                 .setFeedforwardConstants(0.70364, 0.21449, 0.0049559)
                //                 .setFeedbackConstants(4.3347, 0.0, 0.15614)
                //                 .build();
        }

        public static final class AutoConstants {
                public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = Constants.SpeedConstants.MAX_SPEED_METERS_PER_SECOND
                                / 2; // During auto, 1/2 of max speed
                public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Constants.SpeedConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
                                / 2; // During auto, 1/2 of max angular speed
                public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECONDSQ = Constants.SpeedConstants.MAX_ACCELERATION_METERS_PER_SECONDSQ
                                / 2; // During auto, 1/2 of max acceleration
                public static final double AUTO_MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ = Constants.SpeedConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECONDSQ
                                / 2; // During auto, 1/2 of max angular acceleration

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
                public static final double JOYSTICK_DEADBAND = 0.13;
        }

}
