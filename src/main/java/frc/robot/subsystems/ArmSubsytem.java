// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel;


import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.PIDGains;

public class ArmSubsytem extends SubsystemBase {
  private CANSparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxPIDController armController;
  private double setpoint;

  private TrapezoidProfile profile;
  private Timer timer;
  
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private double manualValue;
  
  /** Creates a new ArmSubsytem. */
  public ArmSubsytem() {
    armMotor = new CANSparkMax(Constants.ArmConstants.kArmCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    armMotor.setInverted(false);
    armMotor.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmConstants.kSoftLimitForward);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmConstants.kSoftLimitReverse);

    armEncoder = armMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    armEncoder.setPositionConversionFactor(Constants.ArmConstants.kPositionFactor);
    armEncoder.setVelocityConversionFactor(Constants.ArmConstants.kVelocityFactor);

    armController = armMotor.getPIDController();
    PIDGains.setSparkMaxGains(armController, Constants.ArmConstants.kArmPositionGains);

    armMotor.burnFlash();

    setpoint = Constants.ArmConstants.kHomePosition;

    timer = new Timer();
    timer.start();
    timer.reset();

    updateMotionProfile();
  }

  public void setTargetPosition(double _setpoint, ClawSusbsystem _claw) {
    if (_setpoint != setpoint) {
      setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(setpoint, 0.0);
    profile = new TrapezoidProfile(Constants.ArmConstants.kArmMotionConstraint, goal, state);
    timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = timer.get();
    if (profile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(setpoint, 0.0);
    }
    else {
      targetState = profile.calculate(elapsedTime);
    }

    feedforward = Constants.ArmConstants.kArmFeedforward.calculate(armEncoder.getPosition()+Constants.ArmConstants.kArmZeroCosineOffset, targetState.velocity);
    armController.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  public void runManual(double _power) {
    //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
    setpoint = armEncoder.getPosition();
    targetState = new TrapezoidProfile.State(setpoint, 0.0);
    profile = new TrapezoidProfile(Constants.ArmConstants.kArmMotionConstraint, targetState, targetState);
    //update the feedforward variable with the newly zero target velocity
    feedforward = Constants.ArmConstants.kArmFeedforward.calculate(armEncoder.getPosition()+Constants.ArmConstants.kArmZeroCosineOffset, targetState.velocity);
    armMotor.set(_power + (feedforward / 12.0));
    manualValue = _power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Final Setpoint",  () -> setpoint, null);
    builder.addDoubleProperty("Position", () -> armEncoder.getPosition(), null);
    builder.addDoubleProperty("Applied Output", () -> armMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Elapsed Time", () -> timer.get(), null);
    /*builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);*/
    builder.addDoubleProperty("Feedforward", () -> feedforward, null);
    builder.addDoubleProperty("Manual Value", () -> manualValue, null);
    //builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    //addChild("Controller", m_controller);
  }
}
