package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
        SwerveConstants.frontLeftDriveMotorPort,
        SwerveConstants.frontLeftTurningMotorPort,
        SwerveConstants.frontLeftDriveMotorReversed,
        SwerveConstants.frontLeftTurningMotorReversed,
        SwerveConstants.frontLeftDriveAbsoluteEncoderPort,
        SwerveConstants.frontLeftDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.frontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
        SwerveConstants.frontRightDriveMotorPort,
        SwerveConstants.frontRightTurningMotorPort,
        SwerveConstants.frontRightDriveMotorReversed,
        SwerveConstants.frontRightTurningMotorReversed,
        SwerveConstants.frontRightDriveAbsoluteEncoderPort,
        SwerveConstants.frontRightDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.frontRightDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backLeft = new SwerveModule(
        SwerveConstants.backLeftDriveMotorPort,
        SwerveConstants.backLeftTurningMotorPort,
        SwerveConstants.backLeftDriveMotorReversed,
        SwerveConstants.backLeftTurningMotorReversed,
        SwerveConstants.backLeftDriveAbsoluteEncoderPort,
        SwerveConstants.backLeftDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.backLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
        SwerveConstants.backRightDriveMotorPort,
        SwerveConstants.backRightTurningMotorPort,
        SwerveConstants.backRightDriveMotorReversed,
        SwerveConstants.backRightTurningMotorReversed,
        SwerveConstants.backRightDriveAbsoluteEncoderPort,
        SwerveConstants.backRightDriveAbsoluteEncoderOffsetRad,
        SwerveConstants.backRightDriveAbsoluteEncoderReversed
    );

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
        new Translation2d(SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0),
        new Translation2d(-SwerveConstants.wheelBase / 2.0, SwerveConstants.trackWidth / 2.0),
        new Translation2d(-SwerveConstants.wheelBase / 2.0, -SwerveConstants.trackWidth / 2.0)
    );

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
            driveKinematics,
            Rotation2d.fromDegrees(0),
            new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            }
    );

    public SwerveSubsystem(){
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch(Exception e){}
        }).start();
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360.0);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return driveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    @Override
    public void periodic(){
        odometer.update(
            getRotation2d(),
            new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        SmartDashboard.putNumber("Absolute Encoder Front Left", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder Front Right", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder Back Left", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Absolute Encoder Back Right", backRight.getAbsoluteEncoderRad());
    }

    public void resetModuleEncoders(){
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setBrakeMode(boolean enabled){
        frontLeft.setBrakeMode(enabled);
        frontRight.setBrakeMode(enabled);
        backLeft.setBrakeMode(enabled);
        backLeft.setBrakeMode(enabled);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.physicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void drive(double xSpeedMetersPerSecond, double ySpeedMetersPerSecond, double zSpeedMetersPerSecond, boolean fieldRelative){
        ChassisSpeeds chassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, zSpeedMetersPerSecond, getRotation2d())
                : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, zSpeedMetersPerSecond);

        setModuleStates(driveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public double getFrontLeftAbsoluteEncoderRad(){
        return frontLeft.getAbsoluteEncoderRad();
    }

    public double getFrontRightAbsoluteEncoderRad(){
        return frontRight.getAbsoluteEncoderRad();
    }

    public double getBackLeftAbsoluteEncoderRad(){
        return backLeft.getAbsoluteEncoderRad();
    }

    public double getBackRightAbsoluteEncoderRad(){
        return backRight.getAbsoluteEncoderRad();
    }
}
