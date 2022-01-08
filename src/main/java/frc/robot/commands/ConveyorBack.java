// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorBack extends WaitCommand {

  private ConveyorSubsystem m_conveyor;

  /** Creates a new ConveyorBack. */
  public ConveyorBack(double seconds, ConveyorSubsystem conveyor) {
    super(seconds);
    addRequirements(conveyor);
    m_conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    m_conveyor.setSpeed(-0.27);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_conveyor.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (super.isFinished()) {
      return true;
    }
    else return false;
  }
}
