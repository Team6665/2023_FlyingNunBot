// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    //Creating the date structure that refer to the 4 motors (and controllers) that run the drivetrain. 
    private final MotorController frontLeftMotor = (MotorController) new WPI_TalonSRX(DriveTrainConstants.FrontLeftMotorPort);
    private final MotorController backLeftMotor = (MotorController) new WPI_TalonSRX(DriveTrainConstants.RearLeftMotorPort);
    private final MotorController frontRightMotor = (MotorController) new WPI_TalonSRX(DriveTrainConstants.FrontRightMotorPort);
    private final MotorController backRightMotor = (MotorController) new WPI_TalonSRX(DriveTrainConstants.RearRightMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
