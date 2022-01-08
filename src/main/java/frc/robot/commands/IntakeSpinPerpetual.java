/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstant;
import frc.robot.Constants.IntakeIndicator;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpinPerpetual extends CommandBase {
  /**
   * Creates a new IntakeSpin.
   */

  private final IntakeSubsystem m_intake;
  private final ConveyorSubsystem m_conveyorSubsystem;
  private final double m_power = IntakeConstant.kMotorSpeed;

  public IntakeSpinPerpetual(IntakeSubsystem intake, ConveyorSubsystem conveyor) {
    m_intake = intake;
    m_conveyorSubsystem = conveyor;

    // Use addRequirements() here to declare subsystem dependencies.
    // Don't include conveyor here as it is not a true dependency - we just check state
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
    return !m_conveyorSubsystem.getTopSensor();
  }
}
