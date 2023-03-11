// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final DriveTrainSubsystem drivetrain = new DriveTrainSubsystem();
  private final CommandXboxController driverController = new CommandXboxController(OIConstants.DriverControllerPort);
  public static final LimelightSubsystem limelight = new LimelightSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Test this to see how it handles. SHOULD (?) make for a smoother driving experience? Maybe?
    drivetrain.setDefaultCommand(new RunCommand(
      () -> 
        drivetrain.driveArcade(
          MathUtil.applyDeadband(-driverController.getLeftY(), Constants.OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(driverController.getRightX()*Constants.DriveTrainConstants.kTurningScale, Constants.OIConstants.kDriveDeadband))
      , drivetrain)
    );

    // If the above doesn't work, this should.
    // drivetrain.setDefaultCommand(
    //   new RunCommand(() -> drivetrain.arcadeDrive(-driverController.getLeftY(), -driverController.getRightX()),drivetrain));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
