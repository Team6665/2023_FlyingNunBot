// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoBasic extends CommandBase {
  private final DriveTrainSubsystem driveSubsystem;
  private final Supplier<Double> speedFunction, turnFunction;

  /** Creates a new AutoBasic. */
  public AutoBasic(DriveTrainSubsystem driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
      this.speedFunction = speedFunction;
      this.turnFunction = turnFunction;
      this.driveSubsystem = driveSubsystem;
      addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      System.out.println("AutoBasic started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speedFunction.get();
        double realTimeTurn = turnFunction.get();

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;
        driveSubsystem.setMotors(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("AutoBasic ended!");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
