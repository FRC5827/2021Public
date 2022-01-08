/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 Team 5827, Code Purple. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
/**
 * A ShooterDefault command that uses the ShooterSubsystem.
 */
public class ShooterDefault extends CommandBase {

    private final ShooterSubsystem m_shooter;

    /**
     * Creates a new ShooterDefault command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterDefault(ShooterSubsystem subsystem) {
        m_shooter = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooter.setSpeedInRPM(0.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
}
