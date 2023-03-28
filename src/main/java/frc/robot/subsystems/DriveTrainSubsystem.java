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

  //private RelativeEncoder leftEncoder;
  //private RelativeEncoder rightEncoder;
  
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

    drive.setDeadband(0.02); // Scales joystick values. 0.02 is the default
    drive.setMaxOutput(Constants.DriveTrainConstants.speedScale);

    //encoderPID = new PIDController(9, 0, 0);

    gyro.reset();


    //Motor specs
    frontLeftMotor.setInverted(Constants.DriveTrainConstants.kFrontLeftInverted);
    frontLeftMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    frontLeftMotor.burnFlash();

    frontRightMotor.setInverted(Constants.DriveTrainConstants.kFrontRightInverted);
    frontRightMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.burnFlash();

    backLeftMotor.setInverted(Constants.DriveTrainConstants.kRearLeftInverted);
    backLeftMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    backLeftMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.burnFlash();

    backRightMotor.setInverted(Constants.DriveTrainConstants.kRearRightInverted);
    backRightMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    backRightMotor.setIdleMode(IdleMode.kCoast);
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

  public void tankDrive(double leftSpeed, double rightSpeed) {
    int leftSign = leftSpeed >= 0 ? 1 : -1; // Checks leftSpeed and gathers whether it is negative or positive
    int rightSign = rightSpeed >= 0 ? 1 : -1; // Checks rightSpeed and gathers whether it is negative or positive

    double leftPower = ((Constants.DriveTrainConstants.speedScale - Constants.DriveTrainConstants.minDrivePower) * Math.abs(leftSpeed) + Constants.DriveTrainConstants.minDrivePower) * leftSign;
    double rightPower = ((Constants.DriveTrainConstants.speedScale - Constants.DriveTrainConstants.minDrivePower) * Math.abs(rightSpeed) + Constants.DriveTrainConstants.minDrivePower) * rightSign;

    drive.tankDrive(leftPower, rightPower); // Calls WPILib DifferentialDrive method tankDrive(LSpeed, RSpeed)
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

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public double getGyroAngle() {
    return gyro.getAngle() % 360.0;
  }

  public double getGyroPitch() {
    return gyro.getPitch();
  }
  
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), leftEncoderDistance, rightEncoderDistance, pose);
  }
  
  public double getAverageEncoderDistance() {
    return (frontLeftEncoder.getPosition() + frontRightEncoder.getPosition()) / 2.0;
  }
  
  public RelativeEncoder getLeftEncoder() {
    return frontLeftEncoder;
  }
  
  public RelativeEncoder getRightEncoder() {
    return frontRightEncoder;
  }
  
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }
  
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }
  
  public double getTurnRate() {
    return -gyro.getRate();
  }

  @Override
  public void periodic() {
    //SubsystemBase native method
    odometry.update(gyro.getRotation2d(), frontLeftEncoder.getPosition(), frontRightEncoder.getPosition());
    
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("Pitch", getGyroPitch());

    SmartDashboard.putNumber("Front Left Encoder Distance", leftEncoderDistance);
    SmartDashboard.putNumber("Front Right Encoder Distance", rightEncoderDistance);
    
    SmartDashboard.putNumber("Front Left Encoder Position", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Encoder Position", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Encoder Position", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Encoder Position", backRightEncoder.getPosition());

    SmartDashboard.putNumber("Front Left Encoder Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Front Right Encoder Velocity", frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("Back Left Encoder Velocity", backLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Back Right Encoder Velocity", backRightEncoder.getVelocity());

    //This method will be called once per scheduler run
  }
}
