/*--------------------------------------------------------------------------------*/
/* Copyright (c) 2018-2021 FIRST amd Team 5827, Code Purple. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code       */
/* must be accompanied by the FIRST BSD license file in the root directory of     */
/* the project.                                                                   */
/*--------------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeIndicator;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.commands.*;
import frc.robot.resources.TrajectoryModifier;
import frc.robot.resources.TrajectoryModifier.pickTrajectory;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private final PneumaticSubsystem m_pneumaticSubsystem = new PneumaticSubsystem();
	private final PhotonVisionSubsystem m_photonVision = new PhotonVisionSubsystem();
	private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
	private final IntakeSubsystem m_intake = new IntakeSubsystem(m_pneumaticSubsystem, m_conveyor);
	private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_pneumaticSubsystem, m_intake);
	private final ShooterSubsystem m_shooter = new ShooterSubsystem();
	private SendableChooser<Boolean> m_velocityDriveSelector;
	private SendableChooser<Double> m_autoDelay;
	private SendableChooser<pickTrajectory> m_autoSelector;
	private boolean m_shouldCancelAutonomous = false;

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

	// Definitions of the various different autonomous commands go here
	public AutonomousCommand crossLineSequence;
	public AutonomousCommand trenchRunSequenceStaticStart;
	public AutonomousCommand trenchRunSequenceUndefinedStart;
	public AutonomousCommand stealBallsSequence;
	public AutonomousCommand pathWeaverBarrelSequence;
    public AutonomousCommand pathWeaverSlalomSequence;
	public AutonomousCommand pathWeaverBounceSequence;


	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {

		// default for WPILib is to output telemetry for all internal objects, which can be very costly
		// from both a memory allocation aspect as well as CPU time due to garbage collection,
		// so disable it here.
		LiveWindow.disableAllTelemetry();

		// Configure the button bindings
		configureButtonBindings();
		
		// Create camera server for usb camera
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		if (RobotBase.isReal()) { 
			// reduce USB and network bandwidth usage
			camera.setFPS(8);
		}

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		m_robotDrive.setDefaultCommand(
				// A split-stick arcade command, with forward/backward controlled by the left
				// hand, and turning controlled by the right.
				new RunCommand(
					() ->
						m_robotDrive.arcadeDrive(
							(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2)),
							 m_driverController.getRawAxis(0)),
					m_robotDrive));

		m_conveyor.setDefaultCommand(new ConveyorCollect(m_conveyor));

		m_shooter.setDefaultCommand(new ShooterDefault(m_shooter));

		initChooser();

		// Generating trajectories can be compute intensive but only happens once at startup when RobotContainer
		// is instantiated in robotInit(), so any delay can be absorbed
		TrajectoryModifier.genAllTrajectories();

		crossLineSequence = new AutonomousCommand(List.of(
													//this.getCalcCurrentPoseCommand(),
													new InstantCommand(() -> m_photonVision.setDriverMode(false), m_photonVision),
													new WaitCommandDashboard(m_autoDelay),
													this.getFollowPathLowGearCommand(TrajectoryModifier.crossLineTrajectory),
													this.getShootAtTargetSequenceCommand()));

		trenchRunSequenceStaticStart = new AutonomousCommand(List.of(
													new InstantCommand(() -> m_photonVision.setDriverMode(false), m_photonVision),
													new WaitCommandDashboard(m_autoDelay),//(m_autoDelay.getSelected().doubleValue())),													this.getShootAtTargetSequenceCommand(),
													this.getShootAtTargetSequenceCommand(),
													this.getTurnTowardsBallCommand(),
													new InstantCommand(() -> m_intake.setIntake(IntakeIndicator.down)),
													this.getDriveTowardsBallCommand(),
													this.getFollowPathLowGearWithIntakeCommand(TrajectoryModifier.trenchRunPhase2SlowTrajectory),
													this.getFollowPathLowGearCommand(TrajectoryModifier.trenchRunPhase3Trajectory),
													this.getTurnCommand(180.0),
													this.getShootAtTargetSequenceCommand(),
													this.getShiftUpCommand()
													));

		trenchRunSequenceUndefinedStart = new AutonomousCommand(List.of(
													new InstantCommand(() -> m_photonVision.setDriverMode(false), m_photonVision),
													new InstantCommand(() -> new WaitCommand(m_autoDelay.getSelected())),
													this.getShootAtTargetSequenceCommand(),
													this.getCalcCurrentPoseCommand(),
													this.getTurnTowardsBallCommand(),
													new InstantCommand(() -> m_intake.setIntake(IntakeIndicator.down)),
													this.getDriveTowardsBallCommand(),
													this.getFollowPathLowGearWithIntakeCommand(TrajectoryModifier.trenchRunPhase2SlowTrajectory),
													this.getFollowPathLowGearCommand(TrajectoryModifier.trenchRunPhase3Trajectory),
													this.getTurnCommand(180.0),
													this.getShootAtTargetSequenceCommand(),
													this.getShiftUpCommand()
													));
	


		stealBallsSequence = new AutonomousCommand(List.of(
													this.getFollowPathLowGearWithIntakeCommand(TrajectoryModifier.stealPhase1Trajectory),
													this.getFollowPathHighGearCommand(TrajectoryModifier.stealPhase2ReverseTrajectory),
													this.getShootAtTargetSequenceCommand()
													));

		pathWeaverBarrelSequence = new AutonomousCommand(List.of(
													this.getFollowPathHighGearCommand(TrajectoryModifier.pathWeaverBarrelTrajectory)
													));

		System.out.printf("Time it will take to traverse barrel run trajectory: %.2f%n", TrajectoryModifier.pathWeaverBarrelTrajectory.getTotalTimeSeconds());

		pathWeaverSlalomSequence = new AutonomousCommand(List.of(
													this.getFollowPathHighGearCommand(TrajectoryModifier.pathWeaverSlalomTrajectory)
													));

		System.out.printf("Time it will take to traverse slalom trajectory: %.2f%n", TrajectoryModifier.pathWeaverSlalomTrajectory.getTotalTimeSeconds());


		pathWeaverBounceSequence = new AutonomousCommand(List.of(
													this.getFollowPathHighGearCommand(TrajectoryModifier.pathWeaverBounceFullTrajectory)
													));

		double traverseTime = TrajectoryModifier.pathWeaverBounceFullTrajectory.getTotalTimeSeconds();

		System.out.printf("Time it will take to traverse bounce trajectory: %.2f%n", traverseTime);

		m_pneumaticSubsystem.shiftDown();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {

		if (RobotBase.isSimulation() || RobotBase.isReal()) {
			DriverStation.getInstance().silenceJoystickConnectionWarning(true);
		}

		// Shoot at target with vision
		new JoystickButton(m_driverController, Button.kB.value).whenPressed(new ShootAtTargetSequence(m_robotDrive, m_photonVision, m_pneumaticSubsystem, m_conveyor, m_shooter).withInterrupt(() -> Math.abs(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2)) > .04));

		// Intake
		new JoystickButton(m_driverController, Button.kA.value)
			.whenHeld(new IntakeSpin(m_intake));

		// Conveyor Belt Dump
		new JoystickButton(m_driverController, Button.kX.value).whenHeld(new ConveyorOut(m_conveyor, 0.30));

		// empty the shooter
		new JoystickButton(m_driverController, Button.kY.value).whenPressed(new RunCommand(
			() ->
				m_shooter.setSpeedInRPM(1000), m_shooter)
			.withInterrupt(() -> Math.abs(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2)) > .04));

		//Shifter
		new JoystickButton(m_driverController, Button.kBumperLeft.value).whenPressed(new ShiftDown(m_pneumaticSubsystem));
		new JoystickButton(m_driverController, Button.kBumperRight.value).whenPressed(new ShiftUp(m_pneumaticSubsystem));

		// Reverse belt
		new JoystickButton(m_driverController, Button.kBack.value).whenHeld(new ConveyorOut(m_conveyor, -0.30));

		// Raise intake
		new JoystickButton(m_driverController, Button.kStickRight.value).whenPressed(new InstantCommand(() -> m_pneumaticSubsystem.intakeUp()));

	}

	public void initChooser() {
		m_autoSelector = new SendableChooser<pickTrajectory>();

		m_autoSelector.setDefaultOption("Cross Line", pickTrajectory.Line);
		m_autoSelector.addOption("Trench Static Start", pickTrajectory.TrenchStaticStart);
		m_autoSelector.addOption("Trench Undefined Start", pickTrajectory.TrenchUndefinedStart);
//		m_autoSelector.addOption("Steal Balls", pickTrajectory.Steal);
//		m_autoSelector.addOption("Barrel (Pathweaver)", pickTrajectory.Barrel);
//		m_autoSelector.addOption("Slalom (Pathweaver)", pickTrajectory.Slalom);
//		m_autoSelector.addOption("Bounce (Pathweaver)", pickTrajectory.Bounce);
		Shuffleboard.getTab("Autonomous").add("Autonomous sequence", m_autoSelector);

		m_velocityDriveSelector = new SendableChooser<Boolean>();
		m_velocityDriveSelector.addOption("False", false);
		m_velocityDriveSelector.setDefaultOption("True", true);
		Shuffleboard.getTab("Autonomous").add("Use encoders for teleop drive", m_velocityDriveSelector);

		m_autoDelay = new SendableChooser<Double>();
		m_autoDelay.addOption("0 second delay", 0.0);
		m_autoDelay.addOption("2 second delay", 2.0);
		m_autoDelay.setDefaultOption("5 second delay", 5.0);
		m_autoDelay.addOption("8 second delay", 8.0);
		m_autoDelay.addOption("10 second delay", 10.0);
		Shuffleboard.getTab("Autonomous").add("Delay before auton", m_autoDelay);
	}

	public void initGyroAndResetPose() {
		m_robotDrive.zeroHeading();
		m_robotDrive.resetOdometry(new Pose2d());
	}

	public void initGyroAndResetPose(Pose2d initialPose) {
		m_robotDrive.zeroHeading();
		m_robotDrive.resetOdometry(initialPose);
	}

	public void setTeleDriveMode() {
		m_robotDrive.setTeleopDriveVelocityMode(m_velocityDriveSelector.getSelected());
	}


	public AutonomousCommand getAutoCommand() {
		m_shouldCancelAutonomous = false;

		switch (m_autoSelector.getSelected()) {
			case Line:
				initGyroAndResetPose(TrajectoryModifier.crossLineTrajectory.getInitialPose());
				return crossLineSequence;
			case TrenchStaticStart:
				if (RobotBase.isSimulation()) {
					initGyroAndResetPose(TrajectoryModifier.trenchRunPhase1Trajectory.getInitialPose());
				}
				if (RobotBase.isReal()) {
					initGyroAndResetPose(Constants.AutoDefinitions.trenchRunPhase1StartPoint);
				}
				return trenchRunSequenceStaticStart;
			case TrenchUndefinedStart:
				if (RobotBase.isSimulation()) {
					initGyroAndResetPose(new Pose2d(TrajectoryModifier.trenchRunPhase1Trajectory.getInitialPose().getTranslation().minus(new Translation2d(0, 1)), new Rotation2d(0.0)));
				}
				if (RobotBase.isReal()) {
					initGyroAndResetPose(Constants.AutoDefinitions.trenchRunPhase1StartPoint);
				}
				return trenchRunSequenceUndefinedStart;
/*
			case Steal:
				initGyroAndResetPose(TrajectoryModifier.stealPhase1Trajectory.getInitialPose());
				return stealBallsSequence;
			case Barrel:
				initGyroAndResetPose(TrajectoryModifier.pathWeaverBarrelTrajectory.getInitialPose());
				return pathWeaverBarrelSequence;
			case Slalom:
				initGyroAndResetPose(TrajectoryModifier.pathWeaverSlalomTrajectory.getInitialPose());
				return pathWeaverSlalomSequence;
			case Bounce:
				initGyroAndResetPose(TrajectoryModifier.pathWeaverBounceFullTrajectory.getInitialPose());
				m_robotDrive.getField().getObject("trajectory").setTrajectory(TrajectoryModifier.pathWeaverBounceFullTrajectory);
				return pathWeaverBounceSequence;
*/
			default:
				// nothing to do -- shouldn't happen
				return new AutonomousCommand(List.of(new WaitCommand(0.0)));
		}
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getFollowPathHighGearCommand(Trajectory trajectoryToFollow) {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		m_pneumaticSubsystem.shiftUp();

		RamseteCommand ramseteCommand_r0 = new RamseteCommand(trajectoryToFollow,
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
				  m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
				  //m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
				  //m_leftReference.setNumber(leftMetersPerSec);
		  
				  //m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
				  //m_rightReference.setNumber(rightMetersPerSec);
				},
		   m_robotDrive);

		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, -3.048);
		return ramseteCommand_r0.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getFollowPathLowGearCommand(Trajectory trajectoryToFollow) {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		m_pneumaticSubsystem.shiftDown();

		RamseteCommand ramseteCommand_r0 = new RamseteCommand(trajectoryToFollow,
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
				  m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
				  //m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
				  //m_leftReference.setNumber(leftMetersPerSec);
		  
				  //m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
				  //m_rightReference.setNumber(rightMetersPerSec);
				},
		   m_robotDrive);

		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, -3.048);
		return ramseteCommand_r0.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
	}


	public Command getFollowPathWithIntakeCommand(Trajectory trajectoryToFollow) {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		System.out.println("AUTONOMOUS COMMAND");

		// set current robot position at postion 0, with relative heading of 0
		// which should match trajectory provided below

		//m_pneumaticSubsystem.ShiftUp();

		RamseteCommand ramseteCommand = new RamseteCommand(trajectoryToFollow,
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
					m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
			
					//m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
					//m_leftReference.setNumber(leftMetersPerSec);
			
					//m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
					//m_rightReference.setNumber(rightMetersPerSec);
				  },
			 m_robotDrive);

  
		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, 0.5);
		ParallelRaceGroup parallelRace = new ParallelRaceGroup(new ConveyorSensor(m_conveyor), new IntakeSpinPerpetual(m_intake, m_conveyor));
		ParallelDeadlineGroup deadlineGroup = new ParallelDeadlineGroup(ramseteCommand, parallelRace);
		return deadlineGroup.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
	}

	public Command getFollowPathLowGearWithIntakeCommand(Trajectory trajectoryToFollow) {

		//var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
		//var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
		//var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
		//var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");

		//System.out.println("AUTONOMOUS COMMAND");

		// set current robot position at postion 0, with relative heading of 0
		// which should match trajectory provided below

		m_pneumaticSubsystem.shiftDown();

		RamseteCommand ramseteCommand = new RamseteCommand(trajectoryToFollow,
				m_robotDrive::getPose,
				new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
				DriveConstants.kDriveKinematics,
				//m_robotDrive::tankDriveVelocity,
				(leftMetersPerSec, rightMetersPerSec) -> {
					m_robotDrive.tankDriveVelocity(leftMetersPerSec, rightMetersPerSec);
			
					//m_leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
					//m_leftReference.setNumber(leftMetersPerSec);
			
					//m_rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
					//m_rightReference.setNumber(rightMetersPerSec);
				  },
			 m_robotDrive);

  
		// Run path following command, then stop at the end.
		//return new DriveDistance(m_robotDrive, 0.5);
		ParallelRaceGroup parallelRace = new ParallelRaceGroup(new ConveyorSensor(m_conveyor), new IntakeSpinPerpetual(m_intake, m_conveyor));
		ParallelDeadlineGroup deadlineGroup = new ParallelDeadlineGroup(ramseteCommand, parallelRace);
		return deadlineGroup.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));//ramseteCommand.andThen(() -> m_robotDrive.tankDriveVelocity(0, 0));
	}


	public Command getIntakeCommand() {
		return new IntakeSpinPerpetual(m_intake, m_conveyor);
	}

	public Command getConveyorCommand() {
		return new ConveyorSensor(m_conveyor);
	}

	public Command getShootAtTargetSequenceCommand() {
		return new ShootAtTargetSequence(m_robotDrive, m_photonVision, m_pneumaticSubsystem, m_conveyor, m_shooter);
	}

	public Command getCalcCurrentPoseCommand() {
		return new InstantCommand(() -> m_robotDrive.calcCurrentPose(m_photonVision.getDistance(), m_photonVision, this), m_robotDrive, m_photonVision);
	}

	public Command getTurnCommand(double degreesToTurn) {
		return new TurnDegrees(m_robotDrive, false, degreesToTurn);
	}

	public Command getTurnTowardsBallCommand() {
		return new TurnDegrees(m_robotDrive, true, 0.0);
	}

	public Command getDriveTowardsBallCommand() {
		return new DriveToBall(m_robotDrive, m_photonVision);
	}

	public Command getShiftUpCommand() {
		return new InstantCommand(() -> m_pneumaticSubsystem.shiftUp());
	}

	public void intakeUp() {
		m_intake.setIntake(IntakeIndicator.up);
	}

	public void setDriveCoastMode() {
		m_robotDrive.setNeutralMode(NeutralMode.Coast);
	}

	public void setDriveBrakeMode() {
		m_robotDrive.setNeutralMode(NeutralMode.Brake);
	}

	public void updateSimVisionWithPose() {
		m_photonVision.simulationUpdate(m_robotDrive.getPose());
	}

	public boolean shouldCancelAutonomousCommand() {
		return (m_shouldCancelAutonomous);
	}

	public void setCancelAutonomousCommand() {
		m_shouldCancelAutonomous = true;
	}

	public void resetCancelAutonomousCommand() {
		m_shouldCancelAutonomous = false;
	}

}
