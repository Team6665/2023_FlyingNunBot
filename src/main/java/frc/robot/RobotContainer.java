// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
<<<<<<< HEAD
=======
import frc.robot.commands.DriveForwardCmd;
>>>>>>> ee0d4f967324467a4ba68583a1c589f3c8e76ec9
import frc.robot.commands.AutoBasic;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ClawSusbsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
  private final DriveTrainSubsystem drivetrain = new DriveTrainSubsystem();
  private final XboxController driverController = new XboxController(OIConstants.DriverControllerPort);
  public static final LimelightSubsystem limelight = new LimelightSubsystem();
  //public static final ArmSubsytem arm = new ArmSubsytem();
  public static final ClawSusbsystem claw = new ClawSusbsystem();
  public static final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static final TurretSubsystem turret = new TurretSubsystem();

  public RobotContainer() {
    configureBindings();

    //drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(-driverController.getLeftY(), -driverController.getRightX()),drivetrain));
      
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(new RunCommand(
      () -> 
      drivetrain.driveArcade(
          MathUtil.applyDeadband(driverController.getLeftY(), Constants.OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(-driverController.getRightX()*Constants.DriveTrainConstants.kTurningScale, Constants.OIConstants.kDriveDeadband))
      , drivetrain)
    );

      //new JoystickButton(driverController, XboxController.Button.kStart.value).

      //drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(-driverController.getLeftY(), -driverController.getRightX()),drivetrain));

      
      //set up gripper open/close
      // new JoystickButton(driverController, XboxController.Button.kA.value).onTrue(new InstantCommand(() -> claw.openGripper()));
      // new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> claw.closeGripper())).onFalse(new InstantCommand(() -> claw.stopGripperMotor()));
  
      //set up turret preset positions
      new JoystickButton(driverController, XboxController.Button.kLeftBumper.value).onTrue(new InstantCommand(() -> turret.rotateCCW())).onFalse(new InstantCommand(() -> turret.stopTurretMotor()));
      new JoystickButton(driverController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> turret.rotateCW())).onFalse(new InstantCommand(() -> turret.stopTurretMotor()));
      
      // new JoystickButton(driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.ArmConstants.kIntakePosition, claw)));
      // new JoystickButton(driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(() -> arm.setTargetPosition(Constants.ArmConstants.kFeederPosition, claw)));
    }

  public Command getAutonomousCommand() {
<<<<<<< HEAD
    // An example command will be run in autonomous
    return new AutoBasic(drivetrain, 0.5, 0.0);
=======
    return new DriveForwardCmd(drivetrain, Constants.DriveTrainConstants.kAutoDriveForwardDistance);

>>>>>>> ee0d4f967324467a4ba68583a1c589f3c8e76ec9
  }

  public void periodic(){
    // try {
    //   Thread.sleep(100);
    // } catch (InterruptedException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }
    //System.out.println(driverController.getLeftY());
    //System.out.println(driverController.getRightX());
  }
}
