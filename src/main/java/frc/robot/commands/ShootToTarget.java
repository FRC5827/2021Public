/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class ShootToTarget extends CommandBase {
  private ShooterSubsystem m_shooter;
  private PhotonVisionSubsystem m_photonVision;
  private Timer m_timer;
  private double m_calculatedRPMBottom;
  private double m_calculatedRPMTop;

  private NetworkTableEntry m_calculatedRPMBottomDashboard = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter calculatedRPMBottom");
  private NetworkTableEntry m_calculatedRPMTopDashboard = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter calculatedRPMTop");

  // TODO: consider adding dependency on the drive subsystem and maintaining yaw when shooting
  // to try and offset any outside disturbance (i.e., other robot pushing us)
  /**
   * Creates a new ShootToTarget.
   */
  public ShootToTarget(ShooterSubsystem shooter, PhotonVisionSubsystem photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_photonVision = photonVision;
    addRequirements(m_shooter, m_photonVision);

    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.  
  @Override
  public void initialize() {
    final double maxRPM = 6000;

    double visionDistance = 0.0;
    double x0 = 0.0, x1 = 0.0, y0 = 0.0, y1 = 0.0;

    m_calculatedRPMBottom = 0.0;
    m_calculatedRPMTop = 0.0;

    // Expects LEDs to have been turned on previously
    // Our photonvision code currently returns distance in inches
    visionDistance = m_photonVision.getDistance();
    
    // use lookup table/linear interpolation/polynomial to set speed based on distance
    // use linear for now based on measurements

    if (visionDistance < 0) { visionDistance = 0.0; }

    if (visionDistance < 90) {
      m_calculatedRPMBottom = 4000.0;
      m_calculatedRPMTop = 6000.0;
    }
    else if (visionDistance <= 120.0) {
      x0 = 82.0;  y0 = 6000.0;
      x1 = 120.0; y1 = 5000.0;
    }
    else if (visionDistance <= 160.0) {
      x0 = 120.0; y0 = 5000.0;
      x1 = 160.0; y1 = 3700.0;
    }
    else if (visionDistance <= 191.0) {
      x0 = 160.0; y0 = 3700.0;
      x1 = 191.0; y1 = 3600.0;
    }
    else if (visionDistance <= 279.0) {
      x0 = 191.0; y0 = 3700.0;
      x1 = 279.0; y1 = 3800.0;
    }
    else {
      x0 = 279.0; y0 = 3800.0;
      x1 = 300.0; y1 = 4000.0;
    }

    // check x0 and x1.  If both are 0, m_calculatedRPMBottom and top were set explicitly above so don't interpolate 
    if (x0 != 0.0 || x1 != 0.0) {
      if ((x1 - x0) == 0)
      {
          m_calculatedRPMBottom = (y0 + y1) / 2;
      }
      else {
        m_calculatedRPMBottom = y0 + (visionDistance - x0) * (y1 - y0) / (x1 - x0);
      }

      // set top wheel RPM equal to bottom
      m_calculatedRPMTop = m_calculatedRPMBottom;
    }

    if (m_calculatedRPMBottom > maxRPM) {
      // don't exceed max RPM
      m_calculatedRPMBottom = maxRPM;
    }

    if (m_calculatedRPMTop > maxRPM) {
      // don't exceed max RPM
      m_calculatedRPMTop = maxRPM;
    }

    if (m_photonVision.getBestTarget() == null) {
      m_calculatedRPMBottom = 415.0;
      m_calculatedRPMTop = 690.0;
    }

    m_calculatedRPMTopDashboard.setDouble(m_calculatedRPMTop);
    m_calculatedRPMBottomDashboard.setDouble(m_calculatedRPMBottom);
    
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_shooter.setSpeedInRPM(m_calculatedRPMBottom, m_calculatedRPMTop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
    m_shooter.setSpeedInRPM(0.0);
    //System.out.printf("ShootToTarget end -- interrupted: %s\n", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(3.5);
  }
}
