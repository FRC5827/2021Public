/*--------------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST and Team 5827, Code Purple. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code       */
/* must be accompanied by the FIRST BSD license file in the root directory of     */
/* the project.                                                                   */
/*--------------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.TurnPIDConstants;

/**
 * Turns the robot by the specified angle in degrees (positive is counter-clockwise)
 * This method uses a PID loop (PIDController) with gyro heading from the drive subsystem
 * to turn the appropriate amount requested
 */
public class TurnDegrees extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final boolean m_turnTowardsTarget;
  private double m_degreesToTurn;
  private double m_target;
  private PIDController m_PIDController;

  /**
   * Creates a new robotTurnDegrees.
   *
   * @param driveSubsystem The DriveSubsystem used by this command.
   * @param turnTowardsTarget Should we turn in the general direction of the target
   * @param degreesToTurn If turnTowardsTarget is false, how many degrees we should turn 
   */
  public TurnDegrees(DriveSubsystem driveSubsystem, boolean turnTowardsTarget, double degreesToTurn) {
    
    m_PIDController = new PIDController(TurnPIDConstants.kpTurnRio, TurnPIDConstants.kiTurnRio, TurnPIDConstants.kdTurnRio);
   
    m_driveSubsystem = driveSubsystem;
    m_degreesToTurn = degreesToTurn;
    m_target = 0.0;
    m_turnTowardsTarget = turnTowardsTarget;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (!m_turnTowardsTarget) {
      m_target = m_driveSubsystem.getHeading() + m_degreesToTurn;
    }
    else {
      // use current pose to turn towards target rather than fixed angle
      // needs known starting location on the field however
      // angle calculation assumes that ball location is between -90 and -180 degrees heading
      Pose2d relPose = m_driveSubsystem.getPose().relativeTo(Constants.AutoDefinitions.trenchRunPhase1EndPoint);
      double hypot = Math.sqrt(
                              (relPose.getX() * relPose.getX()) + 
                              (relPose.getY() * relPose.getY())
                              );
      // sin(A) / x = sin(90) / hypot
      // A = asin(x / hypot)
      double angle = Math.asin(Math.abs(relPose.getX()) / hypot);
      angle = Math.toDegrees(angle);
      angle = -90.0 - angle;
      System.out.println("hypot: " + hypot);
      System.out.println("relPose: " + relPose);
      m_target = angle;
      System.out.println("m_target: " + m_target);
    }

    m_target = Math.IEEEremainder(m_target, 360);

    m_PIDController.reset();
    m_PIDController.enableContinuousInput(-180.0, 180.0);
    m_PIDController.setTolerance(2.0);
    m_PIDController.setSetpoint(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // super's execute calls PIDController's calculate and sends output to the consumer
  // that was specified in the constructor
  // Since there is nothing here except for super.execute(), this method could be commented out
  @Override
  public void execute() {
    double velocity = m_PIDController.calculate(m_driveSubsystem.getHeading());
    m_driveSubsystem.tankDriveVelocity(-velocity, velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.tankDriveVelocity(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_PIDController.atSetpoint());
  }

  public double getTarget() {
    return m_target;
  }
}
