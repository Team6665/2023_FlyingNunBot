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
    public static final int FrontLeftMotorPort = 1;
    public static final int RearLeftMotorPort = 0;
    public static final int FrontRightMotorPort = 3;
    public static final int RearRightMotorPort = 2;

    public static final int kP = 1;
    public static final int kI = 1;
    public static final int kD = 1;
    
    public static final int integral = 0;
    public static final int previous_error = 0;
    public static final int setpoint = 0;

    public static final int kCurrentLimit = 55;
    public static final double kTurningScale = 0.5;
  }



}
