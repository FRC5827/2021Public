/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import static frc.robot.Constants.TurnPIDConstants;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * An example command that uses an example subsystem.
 */
public class TurnToTarget extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final PhotonVisionSubsystem m_photonVisionSubsystem;
  private PIDController m_turnPIDController;
  private int m_counter;
  private NetworkTableEntry m_turnP_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetP");
  private NetworkTableEntry m_turnI_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetI");
  private NetworkTableEntry m_turnD_NTEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("TurnTargetD");

  /**
   * Creates a new robotTurnDegrees.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param photonVisionSubsystem The PhotonVisionSubsystem used by this command.
   */
  public TurnToTarget(DriveSubsystem driveSubsystem, PhotonVisionSubsystem photonVisionSubsystem) {

    m_driveSubsystem = driveSubsystem;
    m_photonVisionSubsystem = photonVisionSubsystem;
    m_counter = 0;

    m_turnP_NTEntry.setDouble(TurnPIDConstants.kpTurnRio);
    m_turnI_NTEntry.setDouble(TurnPIDConstants.kiTurnRio);
    m_turnD_NTEntry.setDouble(TurnPIDConstants.kdTurnRio);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    addRequirements(photonVisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double kpModifier = 0.0; 
    double kiModifier = 0.0;


    m_photonVisionSubsystem.setPipeline(1);

    double distance = m_photonVisionSubsystem.getDistance();

    if (distance > 150) {
    // TODO: consider adjusting kpTurnRio and/or kdTurnRio depending on distance to target, as smaller inputs will result in larger outputs

    //kpModifier = -(TurnPIDConstants.kpTurnRio / 2);
    //kiModifier = 0.0;
    }

    double p = m_turnP_NTEntry.getDouble(0.0);
    double i = m_turnI_NTEntry.getDouble(0.0);
    double d = m_turnD_NTEntry.getDouble(0.0);

    System.out.println("TurnP: " + p);
    System.out.println("TurnI: " + i);
    System.out.println("TurnD: " + d);

    // consider implementing a range of tolerances based on distance to target
    double tolerance = 1.0;
    
    m_turnPIDController = new PIDController(m_turnP_NTEntry.getDouble(0.0) + kpModifier, m_turnI_NTEntry.getDouble(0.0), m_turnD_NTEntry.getDouble(0.0) + kiModifier);
    m_turnPIDController.setTolerance(tolerance);

    m_turnPIDController.reset();
    m_turnPIDController.disableContinuousInput();
    m_turnPIDController.setSetpoint(0.0);

    m_counter = 0;
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_photonVisionSubsystem.getBestTarget() != null) {
      double velocity = m_turnPIDController.calculate(m_photonVisionSubsystem.getBestTarget().getYaw());
      m_driveSubsystem.tankDriveVelocity(-velocity, velocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // leave LED on as we probably will use in subsequent command or change pipeline
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
  }

  // Returns true when the command should end.
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

    // Has PID loop reached it's setpoint?  If so, we're done.
    return (m_turnPIDController.atSetpoint());
  }
}
