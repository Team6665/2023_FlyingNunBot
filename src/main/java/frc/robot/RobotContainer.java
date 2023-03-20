// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ClawSusbsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private final DriveTrainSubsystem drivetrain = new DriveTrainSubsystem();
  private final XboxController driverController = new XboxController(OIConstants.DriverControllerPort);
  public static final LimelightSubsystem limelight = new LimelightSubsystem();
  // public static final ArmSubsytem arm = new ArmSubsytem();
  // public static final ClawSusbsystem claw = new ClawSusbsystem();
  // public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
  // public static final TurretSubsystem turret = new TurretSubsystem();

  public RobotContainer() {
    configureBindings();

    //drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(-driverController.getLeftY(), -driverController.getRightX()),drivetrain));
      
  }

  private void configureBindings() {
    //Test this to see how it handles. SHOULD (?) make for a smoother driving experience? Maybe?
    // drivetrain.setDefaultCommand(new RunCommand(
    //   () -> 
    //     drivetrain.driveArcade(
    //       MathUtil.applyDeadband(-driverController.getLeftY(), Constants.OIConstants.kDriveDeadband),
    //       MathUtil.applyDeadband(driverController.getRightX()*Constants.DriveTrainConstants.kTurningScale, Constants.OIConstants.kDriveDeadband))
    //   , drivetrain)
    // );

    drivetrain.setDefaultCommand(new RunCommand(
      () -> 
      drivetrain.driveArcade(
          MathUtil.applyDeadband(-driverController.getLeftY(), Constants.OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(driverController.getRightX()*Constants.DriveTrainConstants.kTurningScale, Constants.OIConstants.kDriveDeadband))
      , drivetrain)
    );

      //set up gripper open/close
      // new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> claw.openGripper())).onFalse(new InstantCommand(() -> claw.closeGripper()));
  
      //set up arm preset positions
      // new JoystickButton(driverController, XboxController.Button.kA.value)
      // .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.ArmConstants.kHomePosition, claw)));
      // new JoystickButton(driverController, XboxController.Button.kX.value)
      // .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.ArmConstants.kScoringPosition, claw)));
      // new JoystickButton(driverController, XboxController.Button.kY.value)
      // .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.ArmConstants.kIntakePosition, claw)));
      // new JoystickButton(driverController, XboxController.Button.kB.value)
      // .onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.ArmConstants.kFeederPosition, claw)));
    }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
