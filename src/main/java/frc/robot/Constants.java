// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//packages robot
package frc.robot;

//imports
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.ModuleMap;

import java.util.Map;

//defines class
public final class Constants {
  public static class OperatorConstants {
    //port for which driving controller is plugged into
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    //in case of joystick drift
    public static final double kDeadband = 0.05;
  }

  //defines swerve class
  public static final class Swerve {
    //width of track and size of wheel base
    public static final double kTrackWidth = Units.inchesToMeters(30);
    public static final double kWheelBase = Units.inchesToMeters(30);

    //position of wheel bases
    public static final Translation2d[] kModuleTranslations = {
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2)
    };
    //offsets for cancoders
    // public static final double frontLeftCANCoderOffset = 353;//CANCoder 0
    // public static final double frontRightCANCoderOffset = 352;//CANCoder 1
    // public static final double backLeftCANCoderOffset = 346;//CANCoder 2
    // public static final double backRightCANCoderOffset = 349;//CANCoder 3 
     public static final double frontLeftCANCoderOffset = 0;//CANCoder 0
    public static final double frontRightCANCoderOffset = 0;//CANCoder 1
    public static final double backLeftCANCoderOffset = 0;//CANCoder 2
    public static final double backRightCANCoderOffset = 0;//CANCoder 3

    //enables swervekinematics
    public static final SwerveDriveKinematics kSwerveKinematics =
            new SwerveDriveKinematics(kModuleTranslations);

    //sets max speed
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

    //pid values
    public static final double kP_X = 0.1;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.1;
    public static final double kD_Y = 0;
    public static final double kP_Theta = 8;
    public static final double kD_Theta = 0;

    //controller constraints
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                    kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

    //defines module class, need to check these values
    public static final class Module {
      //gear ratio of drive motor and turn motor
      public static final double kDriveMotorGearRatio = 7.0;
      public static final double kTurningMotorGearRatio = 13.333;

      //diameter of wheels
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);

      //ticks per rotation
      public static final int kNeoCPR = 42;
      public static final int kFalconEncoderCPR = 2048;

      //defines gearboxes
      public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
      public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);

      //distance per tick
      public static final double kDriveDistancePerPulse =
              (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveMotorGearRatio);

      //converting units to meters, m/sec, and degrees
      public static final double kDriveRevToMeters =
              ((kWheelDiameterMeters * Math.PI) / kDriveMotorGearRatio);
      public static final double kDriveRpmToMetersPerSecond =
              kDriveRevToMeters / 60.0;
      public static final double kTurnRotationsToDegrees =
              360.0 / kTurningMotorGearRatio;

      //volt seconds per meter
      public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
      public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
      public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

      //volt seconds per radian
      public static final double kvTurnVoltSecondsPerRadian = 1.47;
      public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348;
    }

    //says where each module is
    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

    //defines port class
    public static final class Ports {
      //motor ports
      public static final int frontLeftDriveMotor = 1;
      public static final int frontLeftTurnMotor = 2;
      public static final int frontRightDriveMotor = 5;
      public static final int frontRightTurnMotor = 6;
      public static final int backLeftDriveMotor = 3;
      public static final int backLeftTurnMotor = 4;
      public static final int backRightDriveMotor = 7;
      public static final int backRightTurnMotor = 8;

      //encoder ports
      public static final int frontLeftBoreEncoder = 2;
      public static final int frontRightBoreEncoder = 8;
      public static final int backLeftBoreEncoder = 4;
      public static final int backRightBoreEncoder = 6;
    }
  }

  // 1 = closed-loop control (using sensor feedback) and 0 = open-loop control (no sensor feedback)
  public enum CONTROL_MODE {
    OPENLOOP,
    CLOSEDLOOP
  }
}