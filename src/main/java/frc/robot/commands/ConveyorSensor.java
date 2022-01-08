/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorSensor extends CommandBase {
  /**
   * Creates a new ConveyerBelt.
   */
  private final ConveyorSubsystem m_conveyorBelt;
  private final Timer timmy = new Timer();
  private boolean ballCheck = false;

  public ConveyorSensor(ConveyorSubsystem conveyorBelt) {
    m_conveyorBelt = conveyorBelt;
    addRequirements(m_conveyorBelt);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_conveyorBelt.getBallSensors() && ballCheck == false){
      ballCheck = true;
      timmy.start();
    }

    if (ballCheck == true && timmy.get() < 0.3 && m_conveyorBelt.getTopSensor()){
      m_conveyorBelt.setSpeed(.45);
    }
    else {
      m_conveyorBelt.setSpeed(0);
      ballCheck = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_conveyorBelt.setSpeed(0);
    timmy.stop();
    timmy.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return timmy.hasElapsed(.37) || !m_conveyorBelt.getTopSensor();
    return false;
  }
}