// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardCmd extends CommandBase {
    private final DriveTrainSubsystem driveSubsystem;
    private final double distance;

    public DriveForwardCmd(DriveTrainSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = driveSubsystem.getAverageEncoderDistance() + distance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveForwardCmd started!");
    }

    @Override
    public void execute() {
        driveSubsystem.setMotors(DriveTrainConstants.kAutoDriveForwardSpeed, DriveTrainConstants.kAutoDriveForwardSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        System.out.println("DriveForwardCmd ended!");
    }

    @Override
    public boolean isFinished() {
        if (driveSubsystem.getAverageEncoderDistance() > distance)
            return true;
        else
            return false;
    }
}
