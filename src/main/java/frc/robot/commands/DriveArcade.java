// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArcade extends CommandBase {
  private final DriveTrainSubsystem drivetrainSubsystem;
  private final XboxController xboxController;

  /** Creates a new DriveArcade. */
  public DriveArcade(DriveTrainSubsystem drivetrainSubsystem, XboxController xboxController) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.xboxController = xboxController;

        addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.arcadeDrive(-xboxController.getRightX(), -xboxController.getLeftY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
