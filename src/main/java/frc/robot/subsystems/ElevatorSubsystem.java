// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.PIDGains;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.util.sendable.SendableBuilder;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorMotor;
  private RelativeEncoder elevatorEncoder;
  private SparkMaxPIDController elevatorController;
  private double setpoint;
  private double prevSetpoint;
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.kElevatorCanID, CANSparkMaxLowLevel.MotorType.kBrushless);
    elevatorMotor.setInverted(false);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setSmartCurrentLimit(Constants.ElevatorConstants.kCurrentLimit);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ElevatorConstants.kSoftLimitForward);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ElevatorConstants.kSoftLimitReverse);

    elevatorEncoder = elevatorMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    elevatorController = elevatorMotor.getPIDController();
    PIDGains.setSparkMaxGains(elevatorController, Constants.ElevatorConstants.kPositionPIDGains);

    elevatorMotor.burnFlash();
    setpoint = Constants.ElevatorConstants.kClosePosition;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
