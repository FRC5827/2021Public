/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutonomousCommand;
import frc.robot.resources.TrajectoryModifier;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.intakeUp();
    m_robotContainer.initGyroAndResetPose();
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setDriveCoastMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    AutonomousCommand autonCommand = m_robotContainer.getAutoCommand();
    m_autonomousCommand = autonCommand;
    if (autonCommand != null) {
      autonCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    if (m_robotContainer.shouldCancelAutonomousCommand()) {
      m_autonomousCommand.cancel();
      m_robotContainer.resetCancelAutonomousCommand();
      m_autonomousCommand = m_robotContainer.getFollowPathLowGearCommand(TrajectoryModifier.crossLineTrajectory);
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {

    // Do not reset gyro and pose here as robot may not be parallel to target
    // which will impact Limelight/photonvision calculations

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
//      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveBrakeMode();
    m_robotContainer.setTeleDriveMode();

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  // This function is called periodically during test mode.
  @Override
  public void testPeriodic() {
  }


  @Override
  public void simulationInit() {
  }

  // This function is called periodically during simulation mode.
  @Override
  public void simulationPeriodic() {
    m_robotContainer.updateSimVisionWithPose();
  }

}
