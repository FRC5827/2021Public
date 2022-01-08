/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 Code Purple 5827 and FIRST. All Rights Reserved.   */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the BSD license file in the root directory of       */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.DriveConstants;


/** DriveToTarget command that uses Drive and PhotonVision subsystems. */
public class DriveToBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonVisionSubsystem m_photonVisionSubsystem;

  private Command m_command;

  public static TrajectoryConfig m_config = new TrajectoryConfig(
              DriveConstants.kMaxSpeedMetersPerSecond_Low,
              DriveConstants.kMaxAccelerationMetersPerSecondSquared_Low)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(new DifferentialDriveVoltageConstraint(
                  new SimpleMotorFeedforward(
                      DriveConstants.ksVolts_Low, DriveConstants.kvVoltSecondsPerMeter_Low,
                      DriveConstants.kaVoltSecondsSquaredPerMeter_Low),
                      DriveConstants.kDriveKinematics, 11.0))
//          .addConstraint(new CentripetalAccelerationConstraint(
//                      DriveConstants.kMaxCentripetalAccelerationMetersPerSecondSq))
          .setEndVelocity(0.77)
  ;


  /**
   * Creates a new DriveToBall.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param photonVisiontSubsystem The PhotonVisionSubsystem used by this command.
   */
  public DriveToBall(DriveSubsystem driveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {

    m_driveSubsystem = driveSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    // m_command command MUST have the same dependency requirements
    addRequirements(driveSubsystem);
    addRequirements(photonVisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // It is up to caller to ensure LEDs are on by enabling or selecting correct pipeline
    //m_photonVisionSubsystem.setLedMode(true);
    //m_photonVisionSubsystem.setPipeline(1);

    m_command = getDriveToBallCommand(m_driveSubsystem, m_photonVisionSubsystem);
    m_command.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
    // leave LED on as we probably will use in subsequent command
    //m_photonVisionSubsystem.setDriverMode(true);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_command.isFinished();
  }

  // prior to calling, make sure we are pointing at the target with yaw from PhotonVision
  // close to zero
  private Command getDriveToBallCommand(DriveSubsystem drive, PhotonVisionSubsystem photonVision) {
    Pose2d currentPose;
    Pose2d startingPose;
    Pose2d endingPose;
    Trajectory trajectory;

    System.out.println("getDriveToBallCommand");

    currentPose = drive.getPose();

    try {
      // use current pose as starting point but flip rotation around as we are reversing through the trajectory points
      startingPose = currentPose;
      System.out.println("startingPose: " + startingPose.toString());
      endingPose = Constants.AutoDefinitions.trenchRunPhase1EndPoint;
      trajectory = TrajectoryGenerator.generateTrajectory(startingPose, List.of(), endingPose, m_config);
      // TODO -- if this works without issue, move to global constant
      return new DriveRamsete(trajectory, drive);
    }
    catch (Exception e) {
      System.out.println("Caught exception with trajectory/Ramsete");
      System.out.println(e.toString());
      return new WaitCommand(0.0);
    }
  }
}
