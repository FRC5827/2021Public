/*---------------------------------------------------------------------------------*/
/* Copyright (c) 2019-2021 FIRST and Team 5827, Code Purple. All Rights Reserved.  */
/* Open Source Software - may be modified and shared by FRC teams. The code        */
/* must be accompanied by the FIRST BSD license file in the root directory of      */
/* the project.                                                                    */
/*---------------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstant;
import frc.robot.Constants.IntakeIndicator;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpin extends CommandBase {
  /**
   * Creates a new IntakeSpin.
   */

  private final IntakeSubsystem m_intake;
  private final double m_power = IntakeConstant.kMotorSpeed; 

  public IntakeSpin(IntakeSubsystem intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntake(IntakeIndicator.down);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_intake.intake(m_power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
