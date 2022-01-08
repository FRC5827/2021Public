/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

//import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPipelineResult;

import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.Constants.TurnPIDConstants;

public class VisionDriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonVisionSubsystem m_photonVisionSubsystem;
  private final PIDController m_turnPIDController;
  private final PIDController m_forwardPIDController;
  private int m_counter;

  /**
   * Creates a new VisionDriveCommand.
   */
  // TODO -- add as global constants
  public VisionDriveCommand(DriveSubsystem drive, PhotonVisionSubsystem photonVision) {
    m_forwardPIDController = new PIDController(.10, 0.000, 0.010);
    m_turnPIDController = new PIDController(TurnPIDConstants.kpTurnRio, TurnPIDConstants.kiTurnRio, TurnPIDConstants.kdTurnRio);

    m_driveSubsystem = drive;
    m_photonVisionSubsystem = photonVision;
    m_counter = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_photonVisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_photonVisionSubsystem.setLedMode(true);
    m_photonVisionSubsystem.setPipeline(1);

    m_forwardPIDController.reset();
    m_forwardPIDController.disableContinuousInput();
    m_forwardPIDController.setTolerance(1.0);
    m_forwardPIDController.setSetpoint(30.0);

    m_turnPIDController.reset();
    m_turnPIDController.disableContinuousInput();
    m_turnPIDController.setTolerance(1.0);
    m_turnPIDController.setSetpoint(0.0);

    m_counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonVisionSubsystem.getBestTarget() != null) {

      PhotonPipelineResult result = m_photonVisionSubsystem.getLatestResult();

      double output = m_forwardPIDController.calculate(m_photonVisionSubsystem.getDistance(result));
      double rotation = m_turnPIDController.calculate(m_photonVisionSubsystem.getBestTarget(result).getYaw());
  
      //output = 0.0;
  
      double outputLeft = output - rotation;
      double outputRight = output + rotation;
  
      m_driveSubsystem.tankDriveVelocity(outputLeft, outputRight);
    }
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
  // We check to see if there is a target, but it may not be visible because the 
  // led's were just enabled, so we keep a counter to check how many times the
  // target wasn't visible.
  @Override
  public boolean isFinished() {
    if (m_photonVisionSubsystem.getBestTarget() == null) {
      m_counter++;
    }
    else {
      m_counter = 0;
    }

    // if we lost target for 12 consecutive scheduled executes, then stop
    if (m_counter >= 12) {
      return true;
    }
    return (m_forwardPIDController.atSetpoint());
  }
}
