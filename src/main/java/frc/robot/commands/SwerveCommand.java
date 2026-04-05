package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

public class SwerveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedSupplier, ySpeedSupplier, zSpeedSupplier;
    private final Supplier<Boolean> fieldOrientedSupplier;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(OperatorConstants.slewRateLimiter);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(OperatorConstants.slewRateLimiter);
    private final SlewRateLimiter zLimiter = new SlewRateLimiter(OperatorConstants.slewRateLimiter);

    public SwerveCommand(
        SwerveSubsystem swerveSubsystem,
        Supplier<Double> xSpeedSupplier,
        Supplier<Double> ySpeedSupplier,
        Supplier<Double> zSpeedSupplier,
        Supplier<Boolean> fieldOrientedSupplier
    ){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.zSpeedSupplier = zSpeedSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        double rawXSpeed = xSpeedSupplier.get();
        double rawYSpeed = ySpeedSupplier.get();
        double rawZSpeed = zSpeedSupplier.get();

        double xSpeed = MathUtil.applyDeadband(rawXSpeed, OperatorConstants.deadband);
        double ySpeed = MathUtil.applyDeadband(rawYSpeed, OperatorConstants.deadband);
        double zSpeed = MathUtil.applyDeadband(rawZSpeed, OperatorConstants.deadband);

        xSpeed = xLimiter.calculate(xSpeed) * SwerveConstants.teleopDriveMaxSpeedMetersPerSecond;
        ySpeed = xLimiter.calculate(ySpeed) * SwerveConstants.teleopDriveMaxSpeedMetersPerSecond;
        zSpeed = xLimiter.calculate(zSpeed) * SwerveConstants.teleopDriveMaxSpeedMetersPerSecond;

        swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, fieldOrientedSupplier.get());
    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }
}
