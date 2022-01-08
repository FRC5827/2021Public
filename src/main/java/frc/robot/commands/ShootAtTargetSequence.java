/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.GearIndicator;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class 
ShootAtTargetSequence extends SequentialCommandGroup {

  private final DriveSubsystem m_driveSubsystem;
  private final PhotonVisionSubsystem m_photonVision;
  private final ShooterSubsystem m_shooter;
  private final PneumaticSubsystem m_pneumatics;
  private final ConveyorSubsystem m_conveyor;
  private GearIndicator m_initialGearState;

  /**
   * Create a new ShootAtTargetSequence command.
   * @param drive The DriveSubsystem this command will run on
   * @param photonVision The PhotonVisionSubsystem this command will run on
   * @param pneumatics The PneumaticSubsystem this command will run on
   * @param conveyor The ConveyorSubsystem this command will run on
   * @param shooter The ShooterSubsystem this command will run on
   */
  public ShootAtTargetSequence(DriveSubsystem drive, PhotonVisionSubsystem photonVision, PneumaticSubsystem pneumatics, ConveyorSubsystem conveyor, ShooterSubsystem shooter) {

    m_photonVision = photonVision;
    m_driveSubsystem = drive;
    m_shooter = shooter;
    m_pneumatics = pneumatics;
    m_conveyor = conveyor;

    addCommands(
      new InstantCommand(() -> photonVision.setPipeline(1)),
      new InstantCommand(() -> photonVision.setDriverMode(false)),
      new ParallelCommandGroup(new TurnToTarget(drive, photonVision),
                               new ConveyorBack(0.15, conveyor)),

      // let vision settle before calling ShootToTarget which calculates distance
      new WaitCommand(0.25),
      new ParallelCommandGroup(new ShootToTarget(shooter, photonVision),
                               new SequentialCommandGroup(new WaitCommand(0.33), new DumpBalls(conveyor, 3.00)))
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialGearState = m_pneumatics.getCurrentGear();
    m_pneumatics.shiftDown();

    super.initialize();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_initialGearState == GearIndicator.high) {
      m_pneumatics.shiftUp();
    }
    else {
      m_pneumatics.shiftDown();
    }


    // TODO: remove the comment if on a real field as we don't want to blind the judges
    //m_limelight.setPipeline(0);
    //m_photonVision.setDriverMode(true);
    if (!interrupted) {
      m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
    }
    super.end(interrupted);
  }

  
}

