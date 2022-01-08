// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitCommandDashboard extends CommandBase {
  protected Timer m_timer = new Timer();
  private double m_duration;
  private SendableChooser<Double> m_autoDelay;

  /** Creates a new WaitCommandDashboard. */
  public WaitCommandDashboard(SendableChooser<Double> autoDelay) {
    m_autoDelay = autoDelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_duration = m_autoDelay.getSelected().doubleValue();
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}

