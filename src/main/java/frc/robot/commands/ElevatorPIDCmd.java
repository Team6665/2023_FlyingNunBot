// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

public class ElevatorPIDCmd extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;
    private SparkMaxPIDController pidController;

    public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem, double setpoint) {
        // this.elevatorSubsystem = elevatorSubsystem;
        // this.pidController = new SparkMaxPIDController(Constants.ElevatorConstants.kElevatorCanID);
        // addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("ElevatorPIDCmd started!");
        //pidController.reset();
    }

    @Override
    public void execute() {
        //double speed = pidController.calculate(elevatorSubsystem.getElevatorSpeed());
        //elevatorSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotor(0);
        System.out.println("ElevatorPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
