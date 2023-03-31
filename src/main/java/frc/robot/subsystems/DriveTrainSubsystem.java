// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI; 

import frc.robot.Constants;
import frc.robot.lib.PIDGains;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrainSubsystem extends SubsystemBase {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private static DifferentialDriveOdometry odometry = null;

  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax backLeftMotor;
  private CANSparkMax backRightMotor;

  private SparkMaxPIDController frontLeftPIDController;
  private SparkMaxPIDController frontRightPIDController;
  private SparkMaxPIDController backLeftPIDController;
  private SparkMaxPIDController backRightPIDController;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder backLeftEncoder;
  private RelativeEncoder backRightEncoder;
  
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  
  private DifferentialDrive drive;
  public static double maxDriverSpeed = Constants.DriveTrainConstants.speedScale;
  public static double leftEncoderDistance;
  public static double rightEncoderDistance;


  public DriveTrainSubsystem() {
    gyro.reset();

    frontLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.DriveTrainConstants.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    backRightMotor  = new CANSparkMax(Constants.DriveTrainConstants.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftMotors = new  MotorControllerGroup(frontLeftMotor,backLeftMotor);
    rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
    
    frontLeftEncoder = frontLeftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    frontRightEncoder = frontRightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    backLeftEncoder = backLeftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    backRightEncoder = backRightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

    frontLeftPIDController = frontLeftMotor.getPIDController();
    frontRightPIDController = frontRightMotor.getPIDController();
    backLeftPIDController = backLeftMotor.getPIDController();
    backRightPIDController = backRightMotor.getPIDController();

    leftEncoderDistance = frontLeftEncoder.getPosition();
    rightEncoderDistance = frontRightEncoder.getPosition();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoderDistance, rightEncoderDistance);
    drive = new DifferentialDrive(leftMotors, rightMotors);

    rightMotors.setInverted(true);
    drive.setMaxOutput(Constants.DriveTrainConstants.speedScale);
    gyro.reset();
    
    //Motor specs
    frontLeftMotor.setInverted(Constants.DriveTrainConstants.kFrontLeftInverted);
    frontLeftMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.burnFlash();

    frontRightMotor.setInverted(Constants.DriveTrainConstants.kFrontRightInverted);
    frontRightMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.burnFlash();

    backLeftMotor.setInverted(Constants.DriveTrainConstants.kRearLeftInverted);
    backLeftMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.burnFlash();

    backRightMotor.setInverted(Constants.DriveTrainConstants.kRearRightInverted);
    backRightMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    backRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.burnFlash();

    //PIDControllerSpecs
    frontLeftPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    frontLeftPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    frontLeftPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    frontLeftPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    frontLeftPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    frontLeftPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    frontLeftPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);
    frontLeftPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);
    frontLeftPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);
    frontLeftPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);

    frontRightPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    frontRightPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    frontRightPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    frontRightPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    frontRightPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    frontRightPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    frontRightPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);
    frontRightPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);
    frontRightPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);
    frontRightPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);

    backLeftPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    backLeftPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    backLeftPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    backLeftPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    backLeftPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    backLeftPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    backLeftPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);
    backLeftPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);
    backLeftPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);
    backLeftPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);

    backRightPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    backRightPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    backRightPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    backRightPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    backRightPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    backRightPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    backRightPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
    backRightPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
    backRightPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
    backRightPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
  }

  public void driveArcade(double _straight, double _turn) {
    double left  = MathUtil.clamp(_straight + _turn, -1.0, 1.0);
    double right = MathUtil.clamp(_straight - _turn, -1.0, 1.0);

    frontLeftMotor.set(left);
    frontRightMotor.set(right);
    backLeftMotor.set(left);
    backRightMotor.set(right);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    // Account for changes in turning when the forward direction changes, if it doesn't work use the one above
    drive.arcadeDrive(xSpeed * maxDriverSpeed, maxDriverSpeed < 0 ? zRotation * maxDriverSpeed : -zRotation * maxDriverSpeed);
  }

  //Initialize Odometry
  public void zeroGyro() {
    gyro.reset();
  }

  public void zeroEncoders() {
    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
  }

  //Speed of Wheels
  public double getRobotSpeed() {
    return (frontLeftEncoder.getVelocity()+frontRightEncoder.getVelocity())/2*(1/10.71)*(2*Math.PI*3)*12;
  }

  //Distance Traveled
  public double getAverageEncoderDistance() {
    return (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) / 2.0*(1/10.71)*(2*Math.PI*3)*12;
  }

  //Angle Turned
  public double getGyroAngle() {
    return gyro.getAngle() % 360.0;
  }

  public double getGyroPitch() {
    return gyro.getPitch();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }
  
  public double getTurnRate() {
    return -gyro.getRate();
  }

  //Resetting Telemetry
  public void zeroHeading() {
    gyro.reset();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), leftEncoderDistance, rightEncoderDistance, pose);
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
  
  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
    
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("Pitch", getGyroPitch());

    SmartDashboard.putNumber("Left Encoder Distance", leftEncoderDistance);
    SmartDashboard.putNumber("Right Encoder Distance", rightEncoderDistance);

    SmartDashboard.putNumber("Left Encoder Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", frontRightEncoder.getVelocity());
  }
}
