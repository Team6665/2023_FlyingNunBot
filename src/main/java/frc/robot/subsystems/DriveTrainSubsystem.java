// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

public class DriveTrainSubsystem extends SubsystemBase {
  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax backLeftMotor;
  private CANSparkMax backRightMotor;

  private SparkMaxPIDController frontLeftPIDController;
  private SparkMaxPIDController frontRightPIDController;
  private SparkMaxPIDController backLeftPIDController;
  private SparkMaxPIDController backRightPIDController;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRighEncoder;
  private RelativeEncoder backLeftEncoder;
  private RelativeEncoder backRighEncoder;

  // WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(DriveTrainConstants.FrontLeftMotorPort);
  // WPI_TalonSRX backLeftMotor =  new WPI_TalonSRX(DriveTrainConstants.RearLeftMotorPort);
  // WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(DriveTrainConstants.FrontRightMotorPort);
  // WPI_TalonSRX backRightMotor = new WPI_TalonSRX(DriveTrainConstants.RearRightMotorPort);
  
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  
  private DifferentialDrive robotDrive;

  public DriveTrainSubsystem() {
    frontLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.kFrontLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontLeftMotor.setInverted(Constants.DriveTrainConstants.kFrontLeftInverted);
    frontLeftMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontLeftMotor.burnFlash();

    frontRightMotor = new CANSparkMax(Constants.DriveTrainConstants.kFrontRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    frontRightMotor.setInverted(Constants.DriveTrainConstants.kFrontRightInverted);
    frontRightMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.burnFlash();

    backLeftMotor = new CANSparkMax(Constants.DriveTrainConstants.kRearLeftCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    backLeftMotor.setInverted(Constants.DriveTrainConstants.kRearLeftInverted);
    backLeftMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.burnFlash();

    backRightMotor  = new CANSparkMax(Constants.DriveTrainConstants.kRearRightCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    backRightMotor.setInverted(Constants.DriveTrainConstants.kRearRightInverted);
    backRightMotor.setSmartCurrentLimit(Constants.DriveTrainConstants.kCurrentLimit);
    backRightMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.burnFlash();

    frontLeftPIDController = frontLeftMotor.getPIDController();
    // frontLeftEncoder = frontLeftMotor.getEncoder();
    // frontLeftPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    // frontLeftPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    // frontLeftPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    // frontLeftPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    // frontLeftPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    // frontLeftPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    // frontLeftPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);
    // frontLeftPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);
    // frontLeftPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);
    // frontLeftPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.frontLeftsmartMotionSlot);

    frontRightPIDController = frontRightMotor.getPIDController();
    // frontRighEncoder = frontRightMotor.getEncoder();
    // frontRightPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    // frontRightPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    // frontRightPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    // frontRightPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    // frontRightPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    // frontRightPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    // frontRightPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);
    // frontRightPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);
    // frontRightPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);
    // frontRightPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.frontRightsmartMotionSlot);

    backLeftPIDController = backLeftMotor.getPIDController();
    // backLeftEncoder = backLeftMotor.getEncoder();
    // backLeftPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    // backLeftPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    // backLeftPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    // backLeftPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    // backLeftPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    // backLeftPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    // backLeftPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);
    // backLeftPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);
    // backLeftPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);
    // backLeftPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.backLeftsmartMotionSlot);

    backRightPIDController = backRightMotor.getPIDController();
    // backRighEncoder = backRightMotor.getEncoder();
    // backRightPIDController.setP(Constants.DriveTrainPIDConstants.kP);
    // backRightPIDController.setI(Constants.DriveTrainPIDConstants.kI);
    // backRightPIDController.setD(Constants.DriveTrainPIDConstants.kD);
    // backRightPIDController.setIZone(Constants.DriveTrainPIDConstants.kIz);
    // backRightPIDController.setFF(Constants.DriveTrainPIDConstants.kFF);
    // backRightPIDController.setOutputRange(Constants.DriveTrainPIDConstants.kMinOutput, Constants.DriveTrainPIDConstants.kMaxOutput);
    // backRightPIDController.setSmartMotionMaxVelocity(Constants.DriveTrainPIDConstants.maxVel, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
    // backRightPIDController.setSmartMotionMinOutputVelocity(Constants.DriveTrainPIDConstants.minVel, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
    // backRightPIDController.setSmartMotionMaxAccel(Constants.DriveTrainPIDConstants.maxAcc, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);
    // backRightPIDController.setSmartMotionAllowedClosedLoopError(Constants.DriveTrainPIDConstants.allowedErr, Constants.DriveTrainPIDConstants.backRightsmartMotionSlot);

    leftMotors = new  MotorControllerGroup(frontLeftMotor,backLeftMotor);
    rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
    
    robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    rightMotors.setInverted(true);
  }

  public void arcadeDrive(double fwd, double rot) {
    robotDrive.arcadeDrive(fwd, rot);
  }

  public void driveArcade(double straight, double turn) {
    // double left  = MathUtil.clamp(straight + turn, -1.0, 1.0);
    // double right = MathUtil.clamp(straight - turn, -1.0, 1.0);

    // //I think this should normalize the output, but I might be just repeating what the motorcontroller code does.
    // //Make sure to test with and without this
    // frontLeftMotor.set(left);
    // frontRightMotor.set(right);
    // backLeftMotor.set(left);
    // backRightMotor.set(right);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Front Left Encoder Position", frontLeftEncoder.getPosition());
    // SmartDashboard.putNumber("Front Right Encoder Position", frontRighEncoder.getPosition());
    // SmartDashboard.putNumber("Back Left Encoder Position", backLeftEncoder.getPosition());
    // SmartDashboard.putNumber("Back Right Encoder Position", backRighEncoder.getPosition());

    // SmartDashboard.putNumber("Front Left Encoder Velocity", frontLeftEncoder.getVelocity());
    // SmartDashboard.putNumber("Front Right Encoder Velocity", frontRighEncoder.getVelocity());
    // SmartDashboard.putNumber("Back Left Encoder Velocity", backLeftEncoder.getVelocity());
    // SmartDashboard.putNumber("Back Right Encoder Velocity", backRighEncoder.getVelocity());

    // This method will be called once per scheduler run
  }
}
