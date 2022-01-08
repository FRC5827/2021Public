/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
    /**
     * Creates a new ConveyerBeltSub.
     */
    private WPI_VictorSPX motor = new WPI_VictorSPX(Constants.ConveyorbeltId.ConveyorbeltMotorID);
    
    private DigitalInput bottomRight = new DigitalInput(Constants.ConveyorbeltId.bottomRightSensorID);
    private DigitalInput bottomLeft = new DigitalInput(Constants.ConveyorbeltId.bottomLeftSensorID);
    private DigitalInput top = new DigitalInput(Constants.ConveyorbeltId.topSensorID);

    public ConveyorSubsystem() {
        motor.configFactoryDefault();
    }

    @Override
    public void periodic() {

    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public boolean getBottomSensors(){
        return (!bottomLeft.get()) || (!bottomRight.get());
    }

    public boolean getBottomLeftSensor() {
        return !bottomLeft.get();
    }

    public boolean getBottomRightSensor() {
        return !bottomRight.get();
    }

    public boolean getTopSensor(){
        return top.get();
    }

    public boolean getBallSensors(){
        return getBottomSensors() && getTopSensor();
    }
}