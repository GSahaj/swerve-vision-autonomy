// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.config.ConfigLoader;

public final class Constants {

    public static final ConfigLoader DRIVE = new ConfigLoader("Swerve.json");
    public static final ConfigLoader OPERATOR = new ConfigLoader("Operator.json");

    public static class OperatorConstants{
        public static final int driverControllerPort = OPERATOR.getInt("operator", "driverControllerPort");
        public static final int driverXAxis = OPERATOR.getInt("operator", "driveXAxis");
        public static final int driverYAxis = OPERATOR.getInt("operator", "driveYAxis");
        public static final int driverZAxis = OPERATOR.getInt("operator", "driveZAxis");
        public static final int fieldOrientedButton = OPERATOR.getInt("operator", "fieldOrientedButton");
        public static final double deadband = OPERATOR.getDouble("operator", "deadband");
        public static final double slewRateLimiter = OPERATOR.getDouble("operator", "slewRateLimiter");
    }

    public static class ModuleConstants {
        public static final double wheelDiameterMeters = Units.inchesToMeters(DRIVE.getDouble("module", "wheelDiameterInches"));
        public static final double driveMotorGearRatio = 1.0 / DRIVE.getDouble("module", "driveGearRatio");
        public static final double turningMotorGearRatio = 1.0 / DRIVE.getDouble("module", "turningGearRatio");
        public static final double driveEncoderRotationToMeter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;
        public static final double turningEncoderRotationToRadians = turningMotorGearRatio * 2.0 * Math.PI;
        public static final double driveEncoderRPMToMeterPerSec = driveEncoderRotationToMeter / 60.0;
        public static final double turningEncoderRPMToRadiansPerSec = turningEncoderRotationToRadians / 60.0;
        public static final double kPTurning = DRIVE.getDouble("module", "turningKP");

        public static final int driveCurrentLimit = DRIVE.getInt("limits", "driveCurrentLimit");
        public static final int turningCurrentLimit = DRIVE.getInt("limits", "turningCurrentLimit");
    }

    public static class SwerveConstants {
        public static final double trackWidth = Units.inchesToMeters(DRIVE.getDouble("dimensions", "trackWidthInches"));
        public static final double wheelBase = Units.inchesToMeters(DRIVE.getDouble("dimensions", "wheelBaseInches"));

        public static final int frontLeftDriveMotorPort = DRIVE.getInt("motors", "drive", "frontLeft");
        public static final int frontRightDriveMotorPort = DRIVE.getInt("motors", "drive", "frontRight");
        public static final int backLeftDriveMotorPort = DRIVE.getInt("motors", "drive", "backLeft");
        public static final int backRightDriveMotorPort = DRIVE.getInt("motors", "drive", "backRight");

        public static final int frontLeftTurningMotorPort = DRIVE.getInt("motors", "turning", "frontLeft");
        public static final int frontRightTurningMotorPort = DRIVE.getInt("motors", "turning", "frontRight");
        public static final int backLeftTurningMotorPort = DRIVE.getInt("motors", "turning", "backLeft");
        public static final int backRightTurningMotorPort = DRIVE.getInt("motors", "turning", "backRight");

        public static final int frontLeftDriveAbsoluteEncoderPort = DRIVE.getInt("encoder", "absolutePorts", "frontLeft");
        public static final int frontRightDriveAbsoluteEncoderPort = DRIVE.getInt("encoder", "absolutePorts", "frontRight");
        public static final int backLeftDriveAbsoluteEncoderPort = DRIVE.getInt("encoder", "absolutePorts", "backLeft");
        public static final int backRightDriveAbsoluteEncoderPort = DRIVE.getInt("encoder", "absolutePorts", "backRight");

        public static final int frontLeftDriveAbsoluteEncoderOffsetRad = DRIVE.getInt("encoder", "absoluteOffsetsRad", "frontLeft");
        public static final int frontRightDriveAbsoluteEncoderOffsetRad = DRIVE.getInt("encoder", "absoluteOffsetsRad", "frontRight");
        public static final int backLeftDriveAbsoluteEncoderOffsetRad = DRIVE.getInt("encoder", "absoluteOffsetsRad", "backLeft");
        public static final int backRightDriveAbsoluteEncoderOffsetRad = DRIVE.getInt("encoder", "absoluteOffsetsRad", "backRight");

        public static final boolean frontLeftDriveMotorReversed = DRIVE.getBoolean("motor", "driveReversed", "frontLeft");
        public static final boolean frontRightDriveMotorReversed = DRIVE.getBoolean("motor", "driveReversed", "frontRight");
        public static final boolean backLeftDriveMotorReversed = DRIVE.getBoolean("motor", "driveReversed", "backLeft");
        public static final boolean backRightDriveMotorReversed = DRIVE.getBoolean("motor", "driveReversed", "backRight");

        public static final boolean frontLeftTurningMotorReversed = DRIVE.getBoolean("motor", "turningReversed", "frontLeft");
        public static final boolean frontRightTurningMotorReversed = DRIVE.getBoolean("motor", "turningReversed", "frontRight");
        public static final boolean backLeftTurningMotorReversed = DRIVE.getBoolean("motor", "turningReversed", "backLeft");
        public static final boolean backRightTurningMotorReversed = DRIVE.getBoolean("motor", "turningReversed", "backRight");

        public static final boolean frontLeftDriveAbsoluteEncoderReversed = DRIVE.getBoolean("encoder", "absoluteEncoderReversed", "frontLeft");
        public static final boolean frontRightDriveAbsoluteEncoderReversed = DRIVE.getBoolean("encoder", "absoluteEncoderReversed", "frontRight");
        public static final boolean backLeftDriveAbsoluteEncoderReversed = DRIVE.getBoolean("encoder", "absoluteEncoderReversed", "backLeft");
        public static final boolean backRightDriveAbsoluteEncoderReversed = DRIVE.getBoolean("encoder", "absoluteEncoderReversed", "backRight");

        public static final double physicalMaxSpeedMetersPerSecond = DRIVE.getDouble("limits", "maxSpeedMps");
        public static final double physicalMaxAngularSpeedRadiansPerSecond = DRIVE.getDouble("limits", "maxAngularRadPerSec");
        public static final double teleopDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMetersPerSecond * DRIVE.getDouble("limits", "teleopSpeedScale");
        public static final double teleopDriveMaxAngularSpeedRadiansPerSecond = physicalMaxAngularSpeedRadiansPerSecond * DRIVE.getDouble("limits", "teleopSpeedScale");
    }
}
