/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drive the robot forward in a straight line for a particular distance using encoder values from the drivetrain
 */
public class DriveDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private double m_distanceToDriveLeft;
  private double m_distanceToDriveRight;
  private int m_positionTargetLeft;
  private int m_positionTargetRight;
 
  /**
   * Creates a new robotTurnDegrees.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param distanceToDrive The distance in meters to drive
   */
  public DriveDistance(DriveSubsystem driveSubsystem, double distanceToDrive) {
    
    m_driveSubsystem = driveSubsystem;

    m_distanceToDriveLeft = distanceToDrive;
    m_distanceToDriveRight = distanceToDrive;
    
    m_positionTargetLeft = 0;
    m_positionTargetRight = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_positionTargetLeft = (int)((m_driveSubsystem.m_leftEncoderPosition.get() + m_distanceToDriveLeft) / DriveConstants.kEncoderDistancePerPulse);
    m_positionTargetRight = (int)((m_driveSubsystem.m_rightEncoderPosition.get() + m_distanceToDriveRight) / DriveConstants.kEncoderDistancePerPulse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.tankDriveToPosition(m_positionTargetLeft, m_positionTargetRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_positionTargetLeft - m_driveSubsystem.m_leftEncoderPositionTicks.get()) < (.01 / DriveConstants.kEncoderDistancePerPulse))
    {
      return true;
    }
    else
    {
      return false;
    }
 }

}
