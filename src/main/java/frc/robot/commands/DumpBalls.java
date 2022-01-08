/*---------------------------------------------------------------------------------*/
/* Copyright (c) 2019-2021 FIRST and Team 5827, Code Purple. All Rights Reserved.  */
/* Open Source Software - may be modified and shared by FRC teams. The code        */
/* must be accompanied by the FIRST BSD license file in the root directory of      */
/* the project.                                                                    */
/*---------------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class DumpBalls extends CommandBase {
  private ConveyorSubsystem m_conveyor;
  private double m_secondsToRun;
  private double m_secondsToPause;
  private double m_secondsToPulse;
  private double m_dumpSpeed;
  private int m_pulseCount;

  private Timer totalTimer = new Timer();
  private Timer pauseTimer = new Timer();
  private Timer pulseTimer = new Timer();

  NetworkTableEntry m_totalTimeEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Dump Total Time");
  NetworkTableEntry m_pauseTimeEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Dump Pause Time");
  NetworkTableEntry m_pulseTimeEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Dump Pulse Time");
  NetworkTableEntry m_dumpSpeedEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Dump Speed");
  NetworkTableEntry m_dumpSpeedOutputEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Dump Speed output");


  /**
   * Creates a new DumpBalls.
   */
  public DumpBalls(ConveyorSubsystem conveyor, double seconds) {
    m_secondsToRun = m_totalTimeEntry.getDouble(seconds);
    m_secondsToPause = m_pauseTimeEntry.getDouble(0.00);//75);
    m_secondsToPulse = m_pulseTimeEntry.getDouble(0.15);
    m_dumpSpeed = m_dumpSpeedEntry.getDouble(0.50);

    m_totalTimeEntry.setDouble(m_secondsToRun);
    m_pauseTimeEntry.setDouble(m_secondsToPause);
    m_pulseTimeEntry.setDouble(m_secondsToPulse);
    m_dumpSpeedEntry.setDouble(m_dumpSpeed);

    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = conveyor;
    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.  
  @Override
  public void initialize() {
    m_secondsToRun = m_totalTimeEntry.getDouble(m_secondsToRun);
    m_secondsToPause = m_pauseTimeEntry.getDouble(m_secondsToPause);
    m_secondsToPulse = m_pulseTimeEntry.getDouble(m_secondsToPulse);
    m_dumpSpeed = m_dumpSpeedEntry.getDouble(m_dumpSpeed);

    m_pulseCount = 0;

    if ((m_secondsToPulse * 3) + (m_secondsToPause * 2) > m_secondsToRun) {
      // run for min time
      m_secondsToRun = (m_secondsToPulse * 3) + (m_secondsToPause * 2);
    }

    totalTimer.reset();
    totalTimer.start();

    pauseTimer.reset();
    pulseTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double secondsToPulse = m_secondsToPulse;

    if (m_pulseCount >= 2) {
      secondsToPulse = 3.0;
    }

    if (pulseTimer.hasElapsed(secondsToPulse)) {
      pauseTimer.start();
      m_conveyor.setSpeed(0.0);
      m_dumpSpeedOutputEntry.setDouble(0.0);

      if (pauseTimer.hasElapsed(m_secondsToPause)) {
        pauseTimer.stop();
        pauseTimer.reset();
        pulseTimer.stop();
        pulseTimer.reset();
        pulseTimer.start();
        m_conveyor.setSpeed(m_dumpSpeed);
        m_dumpSpeedOutputEntry.setDouble(m_dumpSpeed);
        m_pulseCount++;
      }
    }
    else {
      pulseTimer.start();
      m_conveyor.setSpeed(m_dumpSpeed);
      m_dumpSpeedOutputEntry.setDouble(m_dumpSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.setSpeed(0.0);
    m_dumpSpeedOutputEntry.setDouble(0.0);

    pulseTimer.stop();
    pulseTimer.reset();
    pauseTimer.stop();
    pauseTimer.reset();
    totalTimer.stop();
    totalTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return totalTimer.hasElapsed(m_secondsToRun);
  }
}
