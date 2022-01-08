/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeIndicator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class Reverse extends CommandBase {
  /**
   * Creates a new Reverse.
   */

  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;

  public Reverse(DriveSubsystem drive, IntakeSubsystem intake) {
    m_drive = drive;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drive.isDrivingReversed()) {
      m_drive.setReverseDrive(false);
      m_intake.setIntake(IntakeIndicator.down);
    } else {
      m_drive.setReverseDrive(true);
      m_intake.setIntake(IntakeIndicator.up);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
