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


    turretEncoder = turretMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    //turretEncoder.setP(Constants.TurretConstants.kP);
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
    turretMotor.set(0.5);
  }

  public void rotateCW() {
    turretMotor.set(-0.5);
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

// public class Turret {

//   // Define Spark motor controller object and port number
//   private Spark turretMotor;

//   // Define Encoder object and port numbers
//   private Encoder turretEncoder;

//   // Define PID controller object and constants
//   private PIDController turretPID;
//   private final double kP = 0.1;
//   private final double kI = 0.0;
//   private final double kD = 0.0;
//   private final double kF = 0.0;

  // Constructor method
  // public Turret(int motorPort, int encoderPortA, int encoderPortB) {
  //   // Create motor controller and encoder objects
  //   turretMotor = new Spark(motorPort);
  //   turretEncoder = new Encoder(encoderPortA, encoderPortB, true, Encoder.EncodingType.k4X);

  //   // Set up encoder and PID controller
  //   turretEncoder.setReverseDirection(true);
  //   turretEncoder.setSamplesToAverage(10);
  //   turretPID = new PIDController(kP, kI, kD, kF, turretEncoder, turretMotor);
  //   turretPID.setAbsoluteTolerance(0.5); // Set PID tolerance to 0.5 degrees
  //   turretPID.setOutputRange(-0.5, 0.5); // Set PID output range to -0.5 to 0.5
  //   turretPID.setSetpoint(0); // Set initial setpoint to 0
  // }

  // // Method to set the turret setpoint
  // public void setSetpoint(double setpoint) {
  //   turretPID.setSetpoint(setpoint);
  // }

  // // Method to get the turret output
  // public double getOutput() {
  //   return turretPID.get();
  // }

  // // Method to reset the turret encoder count
  // public void resetEncoder() {
  //   turretEncoder.reset();
  // }

// }

