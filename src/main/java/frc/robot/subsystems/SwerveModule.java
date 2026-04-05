package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turningMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMaxConfig turningConfig;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    private final PIDController turningPIDcontroller;
    private AnalogInput absoluteEncoder;
    private boolean driveMotorReversed;
    private boolean turningMotorReversed;
    private boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(
        int driveMotorID,
        int turningMotorID,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        int absoluteEncoderID,
        double absoluteEncoderOffset,
        boolean absoluteEncoderReversed
    ){
        this.driveMotorReversed = driveMotorReversed;
        this.turningMotorReversed = turningMotorReversed;
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        driveMotor = new SparkMax(driveMotorID, SparkLowLevel.MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, SparkLowLevel.MotorType.kBrushless);

        driveConfig = new SparkMaxConfig();
        turningConfig = new SparkMaxConfig();

        driveConfig
            .inverted(driveMotorReversed)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ModuleConstants.driveCurrentLimit);

        driveConfig.encoder
            .positionConversionFactor(ModuleConstants.driveEncoderRotationToMeter)
            .velocityConversionFactor(ModuleConstants.driveEncoderRPMToMeterPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        turningConfig
            .inverted(turningMotorReversed)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ModuleConstants.turningCurrentLimit);

        turningConfig.encoder
            .positionConversionFactor(ModuleConstants.turningEncoderRotationToRadians)
            .velocityConversionFactor(ModuleConstants.turningEncoderRPMToRadiansPerSec)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        turningPIDcontroller = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPIDcontroller.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoder();
    }

    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoder(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        state.optimize(getTurningRotation());

        double speed = state.speedMetersPerSecond;

        if (Math.abs(speed) < 0.001){
            stop();
            return;
        }

        driveMotor.set(speed / SwerveConstants.physicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPIDcontroller.calculate(getTurningPosition(), state.angle.getRadians()));

    }

    public Rotation2d getTurningRotation(){
        return Rotation2d.fromRadians(getTurningPosition());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setBrakeMode(boolean enabled){
        IdleMode idleMode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
        driveConfig.idleMode(idleMode);
        turningConfig.idleMode(idleMode);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getTurningRotation());
    }

}
