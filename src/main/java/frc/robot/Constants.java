// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


public final class Constants {
  public static class OIConstants {
    public static final int DriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class DriveTrainConstants {
    public static final int kFrontLeftCanId = 15;
    public static final int kRearLeftCanId = 14;
    public static final int kFrontRightCanId = 13;
    public static final int kRearRightCanId = 12;

    public static final boolean kFrontLeftInverted = true;
    public static final boolean kFrontRightInverted = false;
    public static final boolean kRearLeftInverted = true;
    public static final boolean kRearRightInverted = false;

    public static final int FrontLeftsmartMotionSlot = 0;
    public static final int FrontRightsmartMotionSlot = 1;
    public static final int RearLeftsmartMotionSlot = 2;
    public static final int RearRightsmartMotionSlot = 3;

    public static final int kP = 1;
    public static final int kI = 1;
    public static final int kD = 1;
    
    public static final int integral = 0;
    public static final int previous_error = 0;
    public static final int setpoint = 0;

    public static final int kCurrentLimit = 55;
    public static final double kTurningScale = 0.5;
  }

  public static class DriveTrainPIDConstants{
    public static final double kP = 5e-5; 
    public static final double kI = 1e-6;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.000156; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;

    public static final double maxVel = 2000; // rpm
    public static final double minVel = 0;
    public static final double maxAcc = 1500;
    public static final double allowedErr = 0;

    public static final int frontLeftsmartMotionSlot = 0;
    public static final int frontRightsmartMotionSlot = 1;
    public static final int backLeftsmartMotionSlot = 2;
    public static final int backRightsmartMotionSlot = 3;
    
  }

}
