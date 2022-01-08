/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 Team 5827, Code Purple. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class ConveyorCollect extends CommandBase {
  /**
   * Creates a new ConveyorCollect command.
   */
  private final ConveyorSubsystem m_conveyor;
  private final Timer m_timer = new Timer();
  private boolean m_timerStarted = false;
  private boolean m_seenBall = false;

  private NetworkTableEntry m_leftSensor = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("BallSensorLeft");
  private NetworkTableEntry m_rightSensor = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("BallSensorRight");


  public ConveyorCollect(ConveyorSubsystem conveyor) {
    m_conveyor = conveyor;
    addRequirements(m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_seenBall = false;
    m_timerStarted = false;
    m_timer.reset();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_leftSensor.setBoolean(m_conveyor.getBottomLeftSensor());
    m_rightSensor.setBoolean(m_conveyor.getBottomRightSensor());

    if (m_seenBall)
    {
      if (!m_conveyor.getBottomSensors()) {
        // ball has gone through lower opening and is no longer present from IR sensor perspective
        if (!m_timerStarted) {
          m_timerStarted = true;
          m_timer.reset();
          m_timer.start();
        }
        else {
          if (m_timer.hasElapsed(0.25)) {
            m_conveyor.setSpeed(0.0);
            m_seenBall = false;
            m_timer.stop();
            m_timerStarted = false;
          }
        }
      }
      else {
        // ball present
        if (m_timerStarted) {
          // ball present after timer was started (ball through) but timer not elapsed, so stop timer and keep
          // conveyor going, waiting for ball to go through
          m_timer.stop();
          m_timerStarted = false;
        }
      }
    }
    else {
      // if a ball is present, start the belt
      if (m_conveyor.getBottomSensors()) {
        m_seenBall = true;
        m_conveyor.setSpeed(.27);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyor.setSpeed(0);
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}