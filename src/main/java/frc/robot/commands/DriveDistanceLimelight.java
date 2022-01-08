/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.util.Units;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import static frc.robot.Constants.DriveConstants;

public class DriveDistanceLimelight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonVisionSubsystem m_photonVisionSubsystem;

  private int m_positionTargetLeft;
  private int m_positionTargetRight;

  /**
   * Creates a new DriveDistanceLimelight.
   */
  public DriveDistanceLimelight(DriveSubsystem drive, PhotonVisionSubsystem photonVision) {
    m_driveSubsystem = drive;
    m_photonVisionSubsystem = photonVision;

    m_positionTargetLeft = 0;
    m_positionTargetRight = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_photonVisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double distance;

    //m_photonVisionSubsystem.setLedMode(true);
    m_photonVisionSubsystem.setPipeline(1);

    if (m_photonVisionSubsystem.getBestTarget() != null) {
      distance = Units.inchesToMeters(m_photonVisionSubsystem.getDistance() - 20.0);
      if (distance < 0.0) {
        distance = 0.0;
      }
    }
    else {
      distance = 0.0;
    }

    // watch add/subtract -- for 2020 robot with limelight facing backawards, subtract distance to go in reverse
    m_positionTargetLeft = (int)((m_driveSubsystem.m_leftEncoderPosition.get() - distance) / DriveConstants.kEncoderDistancePerPulse);
    m_positionTargetRight = (int)((m_driveSubsystem.m_rightEncoderPosition.get() - distance) / DriveConstants.kEncoderDistancePerPulse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.tankDriveToPosition(m_positionTargetLeft, m_positionTargetRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // caller of command needs to disable LEDs
    //m_photonVisionSubsystem.setLedMode(false);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if within .01 meters of target, we're done
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
