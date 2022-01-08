/*---------------------------------------------------------------------------------*/
/* Copyright (c) 2018-2021 FIRST and Team 5827, Code Purple. All Rights Reserved.  */
/* Open Source Software - may be modified and shared by FRC teams. The code        */
/* must be accompanied by the FIRST BSD license file in the root directory of      */
/* the project.                                                                    */
/*---------------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

import edu.wpi.first.wpilibj.util.Units;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class ConveyorbeltId {
    public static final int ConveyorbeltMotorID = 7;
    public static final int JoystickIDSpeed = 2;
    public static final int bottomRightSensorID = 9;
    public static final int bottomLeftSensorID = 8;
    public static final int topSensorID = 0;
    public static final int JoystickIDRollaRoda = 3;
  }

  public static final int kSlotIdx = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
   * we just want the primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  /**
   * Choose based on what direction you want to be positive, this does not affect
   * motor invert.
   */
  public static boolean kMotorInvert = false;


  public static final class ShooterConstants {
    public static final int kShooterMotor1 = 32;
    public static final int kShooterMotor2 = 33;
  }

  public static final class IntakeConstant {
    public static final int kMotor = 9;
    public static final double kMotorSpeed = 0.3;
  }

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 2;
    public static final int kLeftMotor2Port = 6;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 4;

    public static final int kLeftEncoderPort = 0;
    public static final int kRightEncoderPort = 0;
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    // represents left side of the gearbox
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 7.5; // encoders are after motors and gearing so this value is the same in low or high gear
    public static final int kEncoderCPR = (int)Math.round(4096.0 * kDriveGearing);  // 30720 ticks on current robot
    public static final double kWheelDiameterMeters = 0.149;//0.153;//0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    public static final double kNominalBatteryVoltage = 12.0;
    public static final double kBatteryCompensationForTalon = 11.0;

    // These characterization values MUST be determined either experimentally or
    // theoretically for *our* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these values for our specific robot.

    // voltage to overcome static friction
    public static final double ksVolts_Low = 1.05;
    // voltage to hold a constant velocity
    public static final double kvVoltSecondsPerMeter_Low = 4.91;
    // voltage needed to generate given acceleration
    public static final double kaVoltSecondsSquaredPerMeter_Low = 0.786;
    // voltage to hold a constant turn velocity - only used in simulation
    public static final double kvVoltSecondsPerRadian_Low = 3.5;
    // voltage needed to generate given turn acceleration - only used in simulation
    public static final double kaVoltSecondsSquaredPerRadian_Low = 0.8;

    // voltage to overcome static friction
    public static final double ksVolts_High = 1.27;
    // voltage to hold a constant velocity
    public static final double kvVoltSecondsPerMeter_High = 2.36;
    // voltage needed to generate given acceleration
    public static final double kaVoltSecondsSquaredPerMeter_High = 1.08;
    // voltage to hold a constant turn velocity - only used in simulation
    public static final double kvVoltSecondsPerRadian_High = 0.99;//1.7;
    // voltage needed to generate given turn acceleration - only used in simulation
    public static final double kaVoltSecondsSquaredPerRadian_High = 0.45;//1.2;


    // As above, this must be tuned for our drive!
    // public static final double kPDriveVel = 15.0; // value from characterization when slave is selected in UI - not used

    public static final double kTrackwidthMeters = 0.6961;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final double kDistancePerDegreeOfTurn = (Math.PI * kTrackwidthMeters) / 360.0;

    // this value should be set in the vicinity of what the actual robot top speed is
    public static final double kMaxSpeedMetersPerSecond_Low = 2.3;
    public static final double kMaxAccelerationMetersPerSecondSquared_Low = 2.0;

    public static final double kMaxSpeedMetersPerSecond_High = 4.0;
    public static final double kMaxAccelerationMetersPerSecondSquared_High = 3.0;

    public static final double kMaxRotationSpeedRadiansPerSecond = Units.degreesToRadians(270);
    public static final double kMaxCentripetalAccelerationMetersPerSecondSq = 0.6;

    public static final double kpTalonDriveVel = 0.15; // ideally tune with phoenix tuner
    public static final double kdTalonDriveVel = 1.5;
    public static final double kTalonOpenLoopRamp = 0.33; // time to ramp or slew to max - used by slew rate limiter
    public static final double kTalonDeadband = 0.02; // less than this output will not drive motor
    public static final boolean kLeftTalonInverted = true;
    public static final boolean kRightTalonInverted = false;

    public static final double kFwdInputDeadband = 0.04;
    public static final double kRotInputDeadband = 0.10;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant_Low =
      LinearSystemId.identifyDrivetrainSystem(
        kvVoltSecondsPerMeter_Low,
        kaVoltSecondsSquaredPerMeter_Low,
        kvVoltSecondsPerRadian_Low,
        kaVoltSecondsSquaredPerRadian_Low);
  
    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant_High =
      LinearSystemId.identifyDrivetrainSystem(
        kvVoltSecondsPerMeter_High,
        kaVoltSecondsSquaredPerMeter_High,
        kvVoltSecondsPerRadian_High,
        kaVoltSecondsSquaredPerRadian_High);
}


  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class AutoDefinitions {

    // Cross Line
    public static List<Translation2d> autoLineWaypoints = List.of();
    public static Pose2d autoLineStartPoint = new Pose2d(0, 0, new Rotation2d(0));
    public static Pose2d autoLineEndPoint   = new Pose2d(-1.5, 0, new Rotation2d(0));
    
    // target location on field in Pose2d format
    public static Pose2d targetPose2dLocation = new Pose2d(15.989, 2.404, new Rotation2d());

    // Trench run phase 1
    public static List<Translation2d> trenchRunPhase1Waypoints = List.of();
    public static Pose2d trenchRunPhase1StartPoint = new Pose2d(12.941, 2.404, new Rotation2d(0.0));
    public static Pose2d trenchRunPhase1EndPoint   = new Pose2d(10.941, 0.705, new Rotation2d(Math.PI));

    // Trench run phase 2
    public static List<Translation2d> trenchRunPhase2SlowWaypoints = List.of();
    public static Pose2d trenchRunPhase2SlowStartPoint = trenchRunPhase1EndPoint;//new Pose2d(10.941, 0.705, new Rotation2d(Math.PI));
    public static Pose2d trenchRunPhase2SlowEndPoint   = new Pose2d(8.200, 0.705, new Rotation2d(Math.PI));

    // Trench run phase 3
    public static List<Translation2d> trenchRunPhase3Waypoints = List.of();
    public static Pose2d trenchRunPhase3StartPoint = trenchRunPhase2SlowEndPoint;//new Pose2d(8.200, 0.705, new Rotation2d(Math.PI));
    public static Pose2d trenchRunPhase3EndPoint   = new Pose2d(11.841, 2.404, new Rotation2d(Math.PI));


    // Steal balls phase 1
    public static List<Translation2d> stealPhase1Waypoints = List.of();
    public static Pose2d stealPhase1StartPoint = new Pose2d(0, 0, new Rotation2d(0));
    public static Pose2d stealPhase1EndPoint   = new Pose2d(2, 0, new Rotation2d(0.785));

    // Steal balls phase 2 - reverse
    public static List<Translation2d> stealPhase2ReverseWaypoints = List.of();
    public static Pose2d stealPhase2ReverseStartPoint = new Pose2d(2, 0, new Rotation2d(0.785));
    public static Pose2d stealPhase2ReverseEndPoint   = new Pose2d(-2, -1.524, new Rotation2d(0));
  }


  public final class LimelightVals {
    // public static final double TARGET_HEIGHT_OFF_GROUND = 89.75; // height of center of target bounding box from ground in inches -- from game manual
    // PhotonVision uses bottom of target as height
    public static final double TARGET_HEIGHT_OFF_GROUND = 81.19;
    
    // https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
    // page 197, 208
    public static final double TARGET_HEIGHT = 98.19 - 81.19;
    public static final double TARGET_WIDTH = 41.30 - 6.70;


    public static final double CAMERA_HEIGHT = 33.25; // height of camera from ground in inches
    public static final double TESTING_TARGET_DISTANCE = 120; // temporary distance between camera and target for calc
                                                              // the camera angle
    public static final double TESTING_CAMERA_ANGLE = 11.33; // temporary camera angle
  }

  public static final class TurnPIDConstants {
    public static final double kpTurnRio = 0.075;
    public static final double kiTurnRio = 0.000;
    public static final double kdTurnRio = 0.0015;
  }

  public static final class ArcadeConstants {
    public static final double kSlewRateSpeedMetersPerSecond_Low = DriveConstants.kMaxSpeedMetersPerSecond_Low
        / DriveConstants.kTalonOpenLoopRamp; // slew rate to reach max
    public static final double kSlewRateSpeedMetersPerSecond_High = DriveConstants.kMaxSpeedMetersPerSecond_High
        / DriveConstants.kTalonOpenLoopRamp; // slew rate to reach max
    public static final double kSlewRateRotationSpeedRadiansPerSecond = DriveConstants.kMaxRotationSpeedRadiansPerSecond
        / DriveConstants.kTalonOpenLoopRamp; // max degrees per second rate of change
  }

  public static final class PneumaticConstants {
    // Pneumatics
    public static final int PCM_ID = 0;

    public static final int SHIFTER_FORWARD_CHANNEL = 2;
    public static final int SHIFTER_BACKWARD_CHANNEL = 3;

    public static final int INTAKE_FORWARD_CHANNEL = 0;
    public static final int INTAKE_BACKWARD_CHANNEL = 1;
  }

  public static enum GearIndicator {
    low, high
  }

  public static enum IntakeIndicator {
    up, down
  }

}
