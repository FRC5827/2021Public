/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020-2021 Team 5827, Code Purple. All Rights Reserved.       */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.resources;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoDefinitions;
import frc.robot.Constants.DriveConstants;


public class TrajectoryModifier {

        public static Trajectory crossLineTrajectory;
        public static Trajectory trenchRunPhase1Trajectory;
        public static Trajectory trenchRunPhase2SlowTrajectory;
        public static Trajectory trenchRunPhase3Trajectory;
        public static Trajectory dumpReverseTrajectory;
        public static Trajectory stealPhase1Trajectory;
        public static Trajectory stealPhase2ReverseTrajectory;
        public static Trajectory pathWeaverBarrelTrajectory;
        public static Trajectory pathWeaverSlalomTrajectory;
        public static Trajectory pathWeaverBounceFullTrajectory;


        public enum pickTrajectory {
                Line, TrenchStaticStart, TrenchUndefinedStart/*, Steal, Barrel, Slalom, Bounce*/ 
        }

        public static TrajectoryConfig configHighGear = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_High,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_High)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        //.addConstraint(new DifferentialDriveVoltageConstraint(
                                        //        new SimpleMotorFeedforward(
                                        //                DriveConstants.ksVolts_High,
                                        //                DriveConstants.kvVoltSecondsPerMeter_High,
                                        //                DriveConstants.kaVoltSecondsSquaredPerMeter_High),
                                        //                DriveConstants.kDriveKinematics, 11))
                                        .addConstraint(new CentripetalAccelerationConstraint(
                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq));

        public static TrajectoryConfig configLowGear = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_Low,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        .addConstraint(new DifferentialDriveVoltageConstraint(
                                                new SimpleMotorFeedforward(
                                                        DriveConstants.ksVolts_Low,
                                                        DriveConstants.kvVoltSecondsPerMeter_Low,
                                                        DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                                                        DriveConstants.kDriveKinematics, 11))
//                                        .addConstraint(new CentripetalAccelerationConstraint(
//                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(false);

        public static TrajectoryConfig configLowGearNonZeroEndVel = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_Low,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        .addConstraint(new DifferentialDriveVoltageConstraint(
                                                new SimpleMotorFeedforward(
                                                        DriveConstants.ksVolts_Low,
                                                        DriveConstants.kvVoltSecondsPerMeter_Low,
                                                        DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                                                        DriveConstants.kDriveKinematics, 11))
//                                        .addConstraint(new CentripetalAccelerationConstraint(
//                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(false)
                                        .setEndVelocity(0.77);


        public static TrajectoryConfig configLowGearSlowNonZeroStartVel = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_Low / 3,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        .addConstraint(new DifferentialDriveVoltageConstraint(
                                                new SimpleMotorFeedforward(
                                                        DriveConstants.ksVolts_Low,
                                                        DriveConstants.kvVoltSecondsPerMeter_Low,
                                                        DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                                                        DriveConstants.kDriveKinematics, 11))
//                                        .addConstraint(new CentripetalAccelerationConstraint(
//                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(false)
                                        .setStartVelocity(0.77);


                                        public static TrajectoryConfig configLowGearNonZeroStartVel = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_Low,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        .addConstraint(new DifferentialDriveVoltageConstraint(
                                                new SimpleMotorFeedforward(
                                                        DriveConstants.ksVolts_Low,
                                                        DriveConstants.kvVoltSecondsPerMeter_Low,
                                                        DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                                                        DriveConstants.kDriveKinematics, 11))
//                                        .addConstraint(new CentripetalAccelerationConstraint(
//                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(false)
                                        .setStartVelocity(1.0);



        public static TrajectoryConfig configLowGearReverse = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_Low,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        .addConstraint(new DifferentialDriveVoltageConstraint(
                                                new SimpleMotorFeedforward(DriveConstants.ksVolts_Low,
                                                        DriveConstants.kvVoltSecondsPerMeter_Low,
                                                        DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                                                        DriveConstants.kDriveKinematics, 11))
//                                        .addConstraint(new CentripetalAccelerationConstraint(
//                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(true);

        public static TrajectoryConfig configHighGearReverse = new TrajectoryConfig(
                                                DriveConstants.kMaxSpeedMetersPerSecond_High,
                                                DriveConstants.kMaxAccelerationMetersPerSecondSquared_High)
                                        // Add kinematics to ensure max speed is actually obeyed
                                        .setKinematics(DriveConstants.kDriveKinematics)
                                        // Apply the voltage constraint
                                        //.addConstraint(new DifferentialDriveVoltageConstraint(
                                        //        new SimpleMotorFeedforward(DriveConstants.ksVolts_High,
                                        //                DriveConstants.kvVoltSecondsPerMeter_High,
                                        //                DriveConstants.kaVoltSecondsSquaredPerMeter_High),
                                        //                DriveConstants.kDriveKinematics, 11))
                                        .addConstraint(new CentripetalAccelerationConstraint(
                                                DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
                                        .setReversed(true);


        public static void genAllTrajectories() {
                // generates trajectories based upon the inputed end points and the interior waypoints

                String trajectoryJSON = "";
                Path trajectoryPath;
                Trajectory pathWeaverBounce1Trajectory;
                Trajectory pathWeaverBounce2Trajectory;
                Trajectory pathWeaverBounce3Trajectory;
                Trajectory pathWeaverBounce4Trajectory;
        

                // cross the line
                crossLineTrajectory = TrajectoryGenerator.generateTrajectory(
                        AutoDefinitions.autoLineStartPoint,
                        AutoDefinitions.autoLineWaypoints,
                        AutoDefinitions.autoLineEndPoint,
                        configLowGearReverse);

                // trench run phase 1
                trenchRunPhase1Trajectory = TrajectoryGenerator.generateTrajectory(
                        AutoDefinitions.trenchRunPhase1StartPoint,
                        AutoDefinitions.trenchRunPhase1Waypoints,
                        AutoDefinitions.trenchRunPhase1EndPoint,
                        configLowGear);

                // trench run phase 2
                trenchRunPhase2SlowTrajectory = TrajectoryGenerator.generateTrajectory(
                        AutoDefinitions.trenchRunPhase2SlowStartPoint,
                        AutoDefinitions.trenchRunPhase2SlowWaypoints,
                        AutoDefinitions.trenchRunPhase2SlowEndPoint,
                        configLowGearSlowNonZeroStartVel);

                // trench run phase 3
                trenchRunPhase3Trajectory = TrajectoryGenerator.generateTrajectory(
                        AutoDefinitions.trenchRunPhase3StartPoint,
                        AutoDefinitions.trenchRunPhase3Waypoints,
                        AutoDefinitions.trenchRunPhase3EndPoint,
                        configLowGearReverse);


                // steal balls phase 1
                stealPhase1Trajectory = TrajectoryGenerator.generateTrajectory(
                        AutoDefinitions.stealPhase1StartPoint,
                        AutoDefinitions.stealPhase1Waypoints,
                        AutoDefinitions.stealPhase1EndPoint,
                        configHighGear);

                // steal balls phase 2
                stealPhase2ReverseTrajectory = TrajectoryGenerator.generateTrajectory(
                        AutoDefinitions.stealPhase2ReverseStartPoint,
                        AutoDefinitions.stealPhase2ReverseWaypoints,
                        AutoDefinitions.stealPhase2ReverseEndPoint,
                        configHighGearReverse);


                // load json files containing pathweaver trajectories
                try {
                        trajectoryJSON = "paths/Barrel.wpilib.json";
                        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        pathWeaverBarrelTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                        

                        trajectoryJSON = "paths/Slalom.wpilib.json";
                        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        pathWeaverSlalomTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);


                        trajectoryJSON = "paths/Bounce1.wpilib.json";
                        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        pathWeaverBounce1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                        trajectoryJSON = "paths/Bounce2.wpilib.json";
                        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        pathWeaverBounce2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                        trajectoryJSON = "paths/Bounce3.wpilib.json";
                        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        pathWeaverBounce3Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                        trajectoryJSON = "paths/Bounce4.wpilib.json";
                        trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                        pathWeaverBounce4Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                        pathWeaverBounceFullTrajectory = pathWeaverBounce1Trajectory.concatenate(pathWeaverBounce2Trajectory)
                                                                                .concatenate(pathWeaverBounce3Trajectory)
                                                                                .concatenate(pathWeaverBounce4Trajectory);

                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
                }

        }
}