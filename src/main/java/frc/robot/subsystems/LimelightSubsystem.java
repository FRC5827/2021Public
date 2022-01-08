/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LimelightVals;

public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable m_LimeLightTable;
  private final NetworkTableEntry m_simLLDistance;

  public LimelightSubsystem() {
    m_LimeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_simLLDistance = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("LL Sim Distance");
    m_simLLDistance.setDouble(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("camera angle", calcCameraAngle(LimelightVals.TESTING_TARGET_DISTANCE));
    SmartDashboard.putNumber("distance", getDistance());
  }

  public double getTX() {
    double tx = m_LimeLightTable.getEntry("tx").getDouble(0);
    return tx;
  }

  public double getTY() {
    double ty = m_LimeLightTable.getEntry("ty").getDouble(0);
    return ty;
  }

  public double getTA() {
    double ta = m_LimeLightTable.getEntry("ta").getDouble(0);
    return ta;
  }

  public double getTS() {
    double ts = m_LimeLightTable.getEntry("ts").getDouble(0);
    return ts;
  }

  public boolean hasValidTarget() {
    double tv = m_LimeLightTable.getEntry("tv").getDouble(0);
    return (tv == 1.0);
  }


  public double getDistance() {
    double ty = m_LimeLightTable.getEntry("ty").getDouble(0);

    double angle = ty + LimelightVals.TESTING_CAMERA_ANGLE;
    double deltaH = LimelightVals.TARGET_HEIGHT - LimelightVals.CAMERA_HEIGHT;

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
      distance = m_simLLDistance.getDouble(0.0);
    }

    return distance;
  }
  
  public double calcCameraAngle(double targetDistance)
  {
        double ty = m_LimeLightTable.getEntry("ty").getDouble(0);

        double deltaH = LimelightVals.TARGET_HEIGHT - LimelightVals.CAMERA_HEIGHT;

        // Calculate the inverse camera angle.
        double cameraAngle = Math.atan(deltaH / targetDistance);

        // Convert this to degrees.
        cameraAngle *= 180.0 / Math.PI;

        // Now that the camera angle is in degrees, 
        //subtract the y offset.
        cameraAngle -= ty;

        // Return this angle.
        return cameraAngle;
    }

    /**
     * Set the pipeline to be used by the limelight.
     * 
     * @param pipeline The pipeline to be used.
     */
    public void setPipeline(int pipeline)
    {
      m_LimeLightTable.getEntry("pipeline").setNumber(pipeline);
      NetworkTableInstance.getDefault().flush();
    }

    /**
     * Get the index of the currently selected pipeline.
     * 
     * @return The identifier of the currently selected pipe.
     */
    public int getSelectedPipeline()
    {
        // Get the identifier of the pipeline.
        int pipeId = m_LimeLightTable.getEntry("pipeline").getNumber(-1).intValue();

        // Return this value.
        return pipeId;
    }

    /**
     * Set the camera mode of the Limelight.
     * 
     * @param driverMode Use as vision processor or as standard driver camera.
     */
    public void setDriverMode(boolean driverMode)
    {
        if (driverMode)
        {
          m_LimeLightTable.getEntry("camMode").setNumber(1);
        }
        else
        {
          m_LimeLightTable.getEntry("camMode").setNumber(0);
        }
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Turns the LEDs on or off.
     * 
     * @param bEnable Enable or disable LEDs on the Limelight.
     */
    public void setLedMode(boolean bEnable)
    {
        // 0 uses current pipeline setting
        // 1 forces off
        // 2 forces blink
        // 3 forces on

        if (bEnable)
        {
          m_LimeLightTable.getEntry("ledMode").setNumber(3);
        }
        else
        {
          m_LimeLightTable.getEntry("ledMode").setNumber(1);
        }
        NetworkTableInstance.getDefault().flush();
    }

    public int getLedMode() {
      return m_LimeLightTable.getEntry("ledMode").getNumber(0).intValue(); 
    }

    /**
     * Enable/disable snapshot mode.
     * Enabling will take 2 snapshots per second and
     * store on the Limelight.
     * 
     * @param bEnable Enable or disable snapshot mode.
     */
    public void setSnapshot(boolean bEnable)
    {
        if (bEnable)
        {
          m_LimeLightTable.getEntry("snapshot").setNumber(1);
        }
        else
        {
          m_LimeLightTable.getEntry("snapshot").setNumber(0);
        }
        NetworkTableInstance.getDefault().flush();

    }

    /**
     * Get the Limelight's reported latency contributed by the pipeline.
     * Returns a value in milliseconds. According to
     * the limelight's documentation, at least 11 milliseconds
     * should be added to the value to account for image captre latency.
     * 
     * @return Latency of the Limelight's pipeline processing.
     */
    public double getLatency()
    {
        return m_LimeLightTable.getEntry("tl").getDouble(0.0);
    }
}
