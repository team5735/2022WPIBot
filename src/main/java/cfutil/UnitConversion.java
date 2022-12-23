package cfutil;

import frc.robot.Constants;

public class UnitConversion {
    // Known constants
    public static final double FALCON_ENCODER_CPR = 2048; // units/rev

    // Conversion methods
    public static double falconToMetersPerSecond(double velocityFalconUnits) {
        return (velocityFalconUnits // in units/100ms
                / UnitConversion.FALCON_ENCODER_CPR // becomes rev/100ms
                / Constants.DrivetrainConstants.MK3_DRIVE_MOTOR_GEAR_RATIO // changes # of rev
                * Constants.DrivetrainConstants.WHEEL_CIRCUMFERENCE) // becomes m/100ms
                * 10; // becomes m/s
    }

    public static double metersPerSecondToFalcon(double velocityMetersPerSecond) {
        return (velocityMetersPerSecond // in m/s
                / 10 // becomes m/100ms
                / Constants.DrivetrainConstants.WHEEL_CIRCUMFERENCE // becomes rev/100ms
                * Constants.DrivetrainConstants.MK3_DRIVE_MOTOR_GEAR_RATIO) // changes # of rev
                * UnitConversion.FALCON_ENCODER_CPR; // becomes units/100ms
    }

    public static double falconToMeters(double distanceFalconUnits) {
        return (distanceFalconUnits // in falcon units
                / UnitConversion.FALCON_ENCODER_CPR // becomes rev
                / Constants.DrivetrainConstants.MK3_DRIVE_MOTOR_GEAR_RATIO // changes # of rev
                * Constants.DrivetrainConstants.WHEEL_CIRCUMFERENCE); // becomes m
    }

    public static double metersToFalcon(double distanceMeters) {
        return (distanceMeters // in m
                / Constants.DrivetrainConstants.WHEEL_CIRCUMFERENCE // becomes rev
                * Constants.DrivetrainConstants.MK3_DRIVE_MOTOR_GEAR_RATIO) // changes # of rev
                * UnitConversion.FALCON_ENCODER_CPR; // becomes falcon units
    }

    // I honest to god took this code from 2022 repo and I have no idea how it works
    public static double absoluteEncoderToRadians(double absEncoderRotations, double offsetRotations) {
        return (absEncoderRotations - offsetRotations) // is in units of "rotations"
                % 1 // if it's 2.65 rotations, this makes it 0.65 rotations
                * (2 * Math.PI); // turns 0.65 rotations into fraction of full rotation in radians
    }

}