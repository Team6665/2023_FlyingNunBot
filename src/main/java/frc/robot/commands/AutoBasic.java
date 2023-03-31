// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoBasic extends CommandBase {
<<<<<<< HEAD
  private final DriveTrainSubsystem driveSubsystem;
  private final double speedFunction, turnFunction;

  /** Creates a new AutoBasic. */
  public AutoBasic(DriveTrainSubsystem driveSubsystem, double speedFunction, double turnFunction) {
      this.speedFunction = speedFunction;
      this.turnFunction = turnFunction;
      this.driveSubsystem = driveSubsystem;
      addRequirements(driveSubsystem);
  }
=======

    private final DriveTrainSubsystem driveSubsystem;
    private final Supplier<Double> speedFunction, turnFunction;
>>>>>>> ee0d4f967324467a4ba68583a1c589f3c8e76ec9

    public AutoBasic(DriveTrainSubsystem driveSubsystem,Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
        this.speedFunction = speedFunction;
        this.turnFunction = turnFunction;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

<<<<<<< HEAD
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speedFunction;
        double realTimeTurn = turnFunction;
=======
    @Override
    public void initialize() {
        System.out.println("ArcadeDriveCmd started!");
    }

    @Override
    public void execute() {
        double realTimeSpeed = speedFunction.get();
        double realTimeTurn = turnFunction.get();
>>>>>>> ee0d4f967324467a4ba68583a1c589f3c8e76ec9

        double left = realTimeSpeed + realTimeTurn;
        double right = realTimeSpeed - realTimeTurn;
        driveSubsystem.setMotors(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ArcadeDriveCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}