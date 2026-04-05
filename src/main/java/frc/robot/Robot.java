// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private SwerveSubsystem swerveSubsystem;
    private Command autonmousCommand;

    @Override
    public void robotInit(){
        robotContainer = new RobotContainer();
        swerveSubsystem = robotContainer.getSwerveSubsystem();
        swerveSubsystem.resetModuleEncoders();
    }

    @Override
    public void robotPeriodic(){
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit(){
        swerveSubsystem.resetModuleEncoders();
        autonmousCommand = robotContainer.getAutonomousCommand();
        if(autonmousCommand != null){
            CommandScheduler.getInstance().schedule(autonmousCommand);
        }
    }

    @Override
    public void teleopInit(){
        swerveSubsystem.resetModuleEncoders();;
        if(autonmousCommand != null){
            autonmousCommand.cancel();
        }
    }
}
