// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem= new SwerveSubsystem();
    private final Joystick driverJoystick = new Joystick(OperatorConstants.driverControllerPort);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(
            new SwerveCommand(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OperatorConstants.driverYAxis),
                () -> driverJoystick.getRawAxis(OperatorConstants.driverXAxis),
                () -> driverJoystick.getRawAxis(OperatorConstants.driverZAxis),
                () -> !driverJoystick.getRawButton(OperatorConstants.fieldOrientedButton)
            )
        );
        configureBindings();
    }

    private void configureBindings() {}

    public SwerveSubsystem getSwerveSubsystem(){
        return swerveSubsystem;
    }
    public Command getAutonomousCommand(){
        return null;
    }
}
