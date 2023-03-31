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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends SubsystemBase {
  private CANSparkMax turretMotor;
  private RelativeEncoder turretEncoder;
  private SparkMaxPIDController turretController;
  private double setpoint;
  private double prevSetpoint;
  
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    turretMotor = new CANSparkMax(Constants.TurretConstants.kTurretID, CANSparkMaxLowLevel.MotorType.kBrushless);
    turretMotor.setInverted(false);
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.setSmartCurrentLimit(Constants.TurretConstants.kCurrentLimit);
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.TurretConstants.kSoftLimitForward);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.TurretConstants.kSoftLimitReverse);

    turretEncoder = turretMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    turretEncoder.setPosition(0.0);

    turretController = turretMotor.getPIDController();
    PIDGains.setSparkMaxGains(turretController, Constants.TurretConstants.kPositionPIDGains);

    turretMotor.burnFlash();
    setpoint = Constants.TurretConstants.kStartPosition;
  }

  public void zeroEncoders() {
    turretEncoder.setPosition(0.0);
  }

  public void rotateCCW() {
    turretMotor.set(Constants.TurretConstants.speedScale);
  }

  public void rotateCW() {
    turretMotor.set((-Constants.TurretConstants.speedScale));
  }

  public void stopTurretMotor() {
    turretMotor.set(0.0);
  }





  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Position", turretEncoder.getPosition());
    SmartDashboard.putNumber("Turret Velocity", turretEncoder.getVelocity());


    // This method will be called once per scheduler run
  }
}
