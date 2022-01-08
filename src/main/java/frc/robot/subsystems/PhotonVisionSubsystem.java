/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.LimelightVals;

public class PhotonVisionSubsystem extends SubsystemBase {


  private final PhotonCamera m_photonCamera;
  private final NetworkTableEntry m_simDistance;
  private final SimVisionSystem m_simVision;

  public PhotonVisionSubsystem() {
    m_photonCamera = new PhotonCamera("limelight");
    m_simDistance = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("PhotonVIsion Sim Distance");
    m_simDistance.setDouble(0.0);

    if (RobotBase.isSimulation()) {
      m_simVision = new SimVisionSystem("limelight",
                                        75.7608, // Limelight camera FOV
                                        LimelightVals.TESTING_CAMERA_ANGLE,
                                        new Transform2d (new Translation2d(0.0, 0.2/*-0.6, 0.2*/), new Rotation2d()), // meters
                                        Units.inchesToMeters(LimelightVals.CAMERA_HEIGHT),
                                        7.0, // maxLEDRangeMeters
                                        320, // cameraResWidth
                                        240, // cameraResHeight
                                        10); // minTargetArea)

      //Pose2d targetPose = new Pose2d(new Translation2d(0.0, 2.3), new Rotation2d()); // meters
      Pose2d targetPose = Constants.AutoDefinitions.targetPose2dLocation;
      double targetHeightAboveGround = Units.inchesToMeters(LimelightVals.TARGET_HEIGHT_OFF_GROUND); // meters
      double targetWidth = Units.inchesToMeters(LimelightVals.TARGET_WIDTH);           // meters
      double targetHeight = Units.inchesToMeters(LimelightVals.TARGET_HEIGHT);          // meters
                                        
      SimVisionTarget newTgt = new SimVisionTarget(targetPose,
                                        targetHeightAboveGround,
                                        targetWidth,
                                        targetHeight);
      
      m_simVision.addSimVisionTarget(newTgt);
    }
    else {
      m_simVision = null;
    }

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("camera angle", calcCameraAngle(LimelightVals.TESTING_TARGET_DISTANCE));

    PhotonPipelineResult result = m_photonCamera.getLatestResult();
    
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      SmartDashboard.putNumber("distance", getDistance(result));
      SmartDashboard.putNumber("target pitch", target.getPitch());
      SmartDashboard.putNumber("target yaw", target.getYaw());
    }
  }


  // This function is called periodically during simulation mode.
  public void simulationUpdate(Pose2d robotPose) {
    m_simVision.processFrame(robotPose);
  }


  public PhotonPipelineResult getLatestResult() {
    return m_photonCamera.getLatestResult();
  }


  /**
   * Get distance to best target in inches.
   */

  public double getDistance() {

    PhotonPipelineResult result = m_photonCamera.getLatestResult();

    // Original algorithm from Limelight docs
    /*
    double angle = pitch + LimelightVals.TESTING_CAMERA_ANGLE;
    double deltaH = LimelightVals.TARGET_HEIGHT_OFF_GROUND - LimelightVals.CAMERA_HEIGHT;

    angle *=  Math.PI / 180.0; // Convert the angle into radians.

    double cotAngle;

    double tanAngle = Math.tan(angle);

    // Prevent division by zero.
    if (tanAngle != 0.0)
    {
        cotAngle = 1 / tanAngle;
    }
    else
    {
        cotAngle = 0;
    }

    // a / o * o gives a. a/o is cot(angle).
    double distance = deltaH * cotAngle;

    if (RobotBase.isSimulation()) {
      distance = m_simDistance.getDouble(0.0);
    }
  */
    return getDistance(result);
  }
  

  /**
   * Get distance to best target in inches.
   * 
   * @param result PhotonPipelineResult to use for calculation
   */

  public double getDistance(PhotonPipelineResult result) {

    // docs indicate to check if result has target prior to getting targets ot null pointer exception could occur
    // see https://docs.photonvision.org/en/latest/docs/programming/photonlib/simple-pipeline-result.html#photon-pipeline-result

    if (!result.hasTargets()) {
      return 0;
    }

    double pitch = result.getBestTarget().getPitch();

    // target needs to be set to "bottom" since we are using actual height of the bottom of the target off the ground

    double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(LimelightVals.CAMERA_HEIGHT),
                                                                  Units.inchesToMeters(LimelightVals.TARGET_HEIGHT_OFF_GROUND),
                                                                  Math.toRadians(LimelightVals.TESTING_CAMERA_ANGLE),
                                                                  Math.toRadians(pitch));

    double distance = Units.metersToInches(distanceMeters);

    return distance;
  }


  public Pose2d getFieldRelativePose(PhotonPipelineResult result, Rotation2d yawRotation2d) {
    if (!result.hasTargets()) {
      return null;
    }

    double pitch = result.getBestTarget().getPitch();
    double yaw = result.getBestTarget().getYaw();

    Pose2d robotpose = PhotonUtils.estimateFieldToRobot(Units.inchesToMeters(LimelightVals.CAMERA_HEIGHT),
                                                                Units.inchesToMeters(LimelightVals.TARGET_HEIGHT_OFF_GROUND),
                                                                Math.toRadians(LimelightVals.TESTING_CAMERA_ANGLE),
                                                                Math.toRadians(pitch),
                                                                Rotation2d.fromDegrees(-yaw),
                                                                yawRotation2d,
                                                                Constants.AutoDefinitions.targetPose2dLocation,
                                                                new Transform2d(new Translation2d(Units.inchesToMeters(8.0), Units.inchesToMeters(3.0)), new Rotation2d()));
    return robotpose;
  }


  public double calcCameraAngle(double targetDistance)
  {
    PhotonPipelineResult result = m_photonCamera.getLatestResult();

    // docs indicate to check if result has target prior to getting targets or null pointer exception could occur
    // see https://docs.photonvision.org/en/latest/docs/programming/photonlib/simple-pipeline-result.html#photon-pipeline-result

    if (!result.hasTargets()) {
      return 0;
    }

    double pitch = result.getBestTarget().getPitch();

    // Be consistent with getDistance above -- target needs to be set to "bottom" (or account for crosshair height if not)
    double deltaH = LimelightVals.TARGET_HEIGHT_OFF_GROUND - LimelightVals.CAMERA_HEIGHT;

    // Calculate the inverse camera angle.
    double cameraAngle = Math.atan(deltaH / targetDistance);

    // Convert this to degrees.
    cameraAngle *= 180.0 / Math.PI;

    // Now that the camera angle is in degrees, 
    //subtract the y offset.
    cameraAngle -= pitch;

    // Return this angle.
    return cameraAngle;
  }


 /**
   * Get the best target from photonvision.
   * 
   */
  public boolean hasVisionTarget()
  {
    PhotonPipelineResult result = m_photonCamera.getLatestResult();

    return result.hasTargets();
  }


  /**
   * Get the best target from photonvision.
   * 
   */
  public PhotonTrackedTarget getBestTarget()
  {
    PhotonPipelineResult result = m_photonCamera.getLatestResult();

    return getBestTarget(result);
  }


 /**
   * Get the best target from photonvision.
   * 
   * @param result PhotonPipelineResult to use for target determination
   */
  public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result)
  {
    // docs indicate to check if result has target prior to getting targets or null pointer exception could occur
    // see https://docs.photonvision.org/en/latest/docs/programming/photonlib/simple-pipeline-result.html#photon-pipeline-result

    if (!result.hasTargets()) {
      return null;
    }

    return result.getBestTarget();
  }

  public Translation2d getTranslation() {
    PhotonPipelineResult result = m_photonCamera.getLatestResult();

    if (!result.hasTargets()) return null;

    double yaw = result.getBestTarget().getYaw();

    double distance = Units.inchesToMeters(getDistance(result));
    
    // We are negating the yaw from the camera from CV (computer vision) conventions to
    // standard mathematical conventions. In standard mathematical conventions, as you
    // turn counter-clockwise, angles become more positive.

    return PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-yaw));
  }


  /**
   * Set the pipeline to be used by photonvision.
   * 
   * @param pipeline The pipeline to be used.
   */
  public void setPipeline(int pipeline)
  {
    m_photonCamera.setPipelineIndex(pipeline);
  }

  /**
   * Get the index of the currently selected pipeline.
   * 
   * @return The identifier of the currently selected pipe.
   */
  public int getSelectedPipeline()
  {
      // Get the identifier of the pipeline.
      int pipeId = m_photonCamera.getPipelineIndex();

      // Return this value.
      return pipeId;
  }

  /**
   * Set photonvision's camera mode.
   * 
   * @param driverMode Use as vision processor or as standard driver camera.
   */
  public void setDriverMode(boolean driverMode)
  {
      if (driverMode)
      {
        m_photonCamera.setDriverMode(true);
      }
      else
      {
        m_photonCamera.setDriverMode(false);
      }
  }

  /**
   * Turns the LEDs on or off.
   * 
   * @param bEnable Enable or disable photonvision LEDs.
   */
  public void setLedMode(boolean bEnable)
  {
      if (bEnable)
      {
        m_photonCamera.setLED(LEDMode.kOn);
      }
      else
      {
        m_photonCamera.setLED(LEDMode.kOff);
      }
  }

  public LEDMode getLedMode() {
    return m_photonCamera.getLEDMode(); 
  }

  /**
   * Take a snapshot from the camera stream.
   * Images will be stored in the photonvision configuration directory of the device.
   * 
   * @param bPostProcessing to indicate if snapshot should be taken prior or after image processing
   */
  public void takeSnapshot(boolean bPostProcessing)
  {
      if (bPostProcessing)
      {
        m_photonCamera.takeOutputSnapshot();
      }
      else
      {
        m_photonCamera.takeInputSnapshot();
      }
  }

  /**
   * Get PhotonVisions's pipeline latency.
   * Returns a value in milliseconds.
   * 
   * @return Latency of photonvision's pipeline processing.
   */
  public double getLatency()
  {
      return m_photonCamera.getLatestResult().getLatencyMillis();
  }
}
