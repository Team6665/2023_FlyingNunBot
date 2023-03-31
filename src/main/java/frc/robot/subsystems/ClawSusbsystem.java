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

public class ClawSusbsystem extends SubsystemBase {
  private CANSparkMax clawMotor;
  private RelativeEncoder clawEncoder;
  private SparkMaxPIDController clawController;
  private double setpoint;
  private double prevSetpoint;

  /** Creates a new ClawSusbsystem. */
  public ClawSusbsystem() {
    clawMotor = new CANSparkMax(Constants.ClawConstants.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    clawMotor.setInverted(false);
    clawMotor.setIdleMode(IdleMode.kBrake);
    clawMotor.setSmartCurrentLimit(Constants.ClawConstants.kCurrentLimit);
    clawMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    clawMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    clawMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ClawConstants.kSoftLimitForward);
    clawMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ClawConstants.kSoftLimitReverse);

    clawEncoder = clawMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    clawController = clawMotor.getPIDController();
    PIDGains.setSparkMaxGains(clawController, Constants.ClawConstants.kPositionPIDGains);

    clawMotor.burnFlash();

    setpoint = Constants.ClawConstants.kClosePosition;
  }

  //Set claw position
  public boolean isSafe() {
    return clawEncoder.getPosition() > Constants.ClawConstants.kSafePosition;
  }

  public void openGripper() {
    setpoint = Constants.ClawConstants.kOpenPosition;
  }

  public void closeGripper() {
    setpoint = Constants.ClawConstants.kClosePosition;
  }

  public void stopGripperMotor(){
    clawMotor.set(0.0);
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    if (setpoint != prevSetpoint) {
      clawController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    }
    prevSetpoint = setpoint;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Setpoint", () -> setpoint, (val) -> setpoint = val);
    builder.addDoubleProperty("Position", () -> clawEncoder.getPosition(), null);
  }
}
