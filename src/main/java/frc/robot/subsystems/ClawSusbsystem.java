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

  /** Creates a new ClawSusbsystem. */
  public ClawSusbsystem() {
    clawMotor = new CANSparkMax(Constants.ClawConstants.kGripperCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    clawMotor.setInverted(false);
    //clawMotor.setIdleMode(IdleMode.kBrake);
    //clawMotor.enableVoltageCompensation(2);
    clawEncoder = clawMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    clawController = clawMotor.getPIDController();

    clawMotor.burnFlash();
  }

  //Set claw position
  public void openGripper() {
    clawMotor.set(0.5);
  }

  public void closeGripper() {
    clawMotor.set(-0.5);
  }

  public void stopGripperMotor(){
    clawMotor.set(0.0);
  }
}
