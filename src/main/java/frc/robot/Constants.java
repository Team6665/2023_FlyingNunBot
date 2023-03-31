// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.lib.PIDGains;


public final class Constants {
  public static class OIConstants {
    public static final int DriverControllerPort = 0;
    public static final double kDriveDeadband = 0.15;
  }

  public static class TurningConstants{
    private static final double kAngleSetpoint = 0.0;
    private static final double kP = 0.005; // propotional turning constant
  }

  //Constants for the Drivetrain motors, encoders, and PID Controllers
  public static final class DriveTrainConstants {
    public static final int kFrontLeftCanId = 15;
    public static final int kRearLeftCanId = 14;
    public static final int kFrontRightCanId = 13;
    public static final int kRearRightCanId = 12;
    
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



    public static final boolean kFrontLeftInverted = true;
    public static final boolean kFrontRightInverted = false;
    public static final boolean kRearLeftInverted = true;
    public static final boolean kRearRightInverted = false;

    public static final int kCurrentLimit = 50;
    public static final double kTurningScale = 0.5;

    public static final double speedScale = 0.6;
    public static final double minDrivePowerTurn = 0.35;

    public static final double minDrivePower = 0.1;
    public static final double maxDriveSpeed = 0.8;
    public static final double maxSpeed = 3;
    public static final double maxAcceleration = 3;

    public static final double wheelDistance = 55; //cm
    public static final double kAutoDriveForwardSpeed = 0;
  }



  public static class TurretConstants{
    public static final int kTurretID = 3;
    public static final int turretGearRatio = 462;
    //All units in revs
    public static final double kSoftLimitReverse = -30;
    public static final double kSoftLimitForward = 30;
    public static final double kStartPosition = 0.0;
    public static final double kOpenPosition = -34.0;
    public static final double kSafePosition = -29.0;

    public static final double kP = 5e-5; 
    public static final double kI = 1e-6;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.000156; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;
    
    public static final int kCurrentLimit = 30;

    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
    public static final double speedScale = 0.8;
  }

  public static class ElevatorConstants{
    public static final int kElevatorCanID = 1;
    public static final double elevatorGearRatio = 100;
    //All units in revs
    public static final double kSoftLimitReverse = -34.0;
    public static final double kSoftLimitForward = 5.0;
    public static final double kClosePosition = 0.0;
    public static final double kOpenPosition = -34.0;
    public static final double kSafePosition = -29.0;
    
    public static final int kCurrentLimit =30;

    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);

    public static final double kP = 1;
    public static final double kI = 1;
    public static final double kD = 1;
    

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

  public static final class ClawConstants {
    public static final int kGripperCanId = 5;
    public static final int clawGearRatio = 4;
    //All units in revs
    public static final double kSoftLimitReverse = -34.0;
    public static final double kSoftLimitForward = 5.0;
    public static final double kClosePosition = 0.0;
    public static final double kOpenPosition = -34.0;
    public static final double kSafePosition = -29.0;
    
    public static final int kCurrentLimit =30;

    public static final PIDGains kPositionPIDGains = new PIDGains(0.2, 0.0, 0.0);
  }

  public static final class ArmConstants {
    public static final int kElbowCanID = 2;//elbow;
    public static final int kShoulderCanID = 4;//

    public static final boolean kArmInverted = false;
    public static final int kCurrentLimit = 40;

    public static final double kSoftLimitReverse = 0.0;
    public static final double kSoftLimitForward = 4.6;

    public static final double kArmGearRatio = 1.0 / (48.0 * 4.0); 
    public static final double kPositionFactor = kArmGearRatio * 2.0 * Math.PI; //multiply SM value by this number and get arm position in radians
    public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
    public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
    public static final double kArmZeroCosineOffset = - Math.PI / 6; //radians to add to converted arm position to get real-world arm position (starts at ~30deg angle)
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(0.0, 0.4, 12.0/kArmFreeSpeed, 0.0);
    public static final PIDGains kArmPositionGains = new PIDGains(0.6, 0.0, 0.0);
    public static final TrapezoidProfile.Constraints kArmMotionConstraint = new TrapezoidProfile.Constraints(2.0, 2.0);

    public static final double kHomePosition = 0.0;
    public static final double kScoringPosition = 3.05;
    public static final double kIntakePosition = 4.52;
    public static final double kFeederPosition = 2.95;
  }

}
