// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;


public class DriveTrainSubsystem extends SubsystemBase {
  // private CANSparkMax m_frontLeftMotor;
  // private CANSparkMax m_frontRightMotor;
  // private CANSparkMax m_rearLeftMotor;
  // private CANSparkMax m_rearRightMotor;

  

  WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(DriveTrainConstants.FrontLeftMotorPort);
  WPI_TalonSRX backLeftMotor =  new WPI_TalonSRX(DriveTrainConstants.RearLeftMotorPort);
  WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(DriveTrainConstants.FrontRightMotorPort);
  WPI_TalonSRX backRightMotor = new WPI_TalonSRX(DriveTrainConstants.RearRightMotorPort);
  
  MotorControllerGroup leftMotors = new  MotorControllerGroup(frontLeftMotor,backLeftMotor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(frontRightMotor, backRightMotor);
  
  DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

  public DriveTrainSubsystem() {
    rightMotors.setInverted(true);
  }

  public void arcadeDrive(double fwd, double rot) {
    robotDrive.arcadeDrive(fwd, rot);
  }

  public void driveArcade(double straight, double turn) {
    double left  = MathUtil.clamp(straight + turn, -1.0, 1.0);
    double right = MathUtil.clamp(straight - turn, -1.0, 1.0);

    //I think this should normalize the output, but I might be just repeating what the motorcontroller code does.
    //Make sure to test with and without this
    frontLeftMotor.set(left);
    frontRightMotor.set(right);
    backLeftMotor.set(left);
    backRightMotor.set(right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
