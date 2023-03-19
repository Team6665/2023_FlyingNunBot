// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;


public class DriveTank extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrainSubsystem drivetrainSubsystem;
    private final XboxController xboxController;

    public DriveTank(DriveTrainSubsystem drivetrainSubsystem, XboxController xboxController) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xboxController = xboxController;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //See WPILib for more extensive instructions
        drivetrainSubsystem.tankDrive(xboxController.getLeftY(),
                xboxController.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
