/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020-2021 Team 5827, Code Purple. All Rights Reserved.       */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeIndicator;

public class IntakeSubsystem extends SubsystemBase {

    WPI_VictorSPX m_intakeMotor;
    Boolean override = false;
    private boolean m_isSpinning = false;

    static PneumaticSubsystem m_pneumaticSubsystem;
    static ConveyorSubsystem m_conveyorSubsystem;

    public IntakeSubsystem(PneumaticSubsystem pneumatics, ConveyorSubsystem conveyor) {
        m_pneumaticSubsystem = pneumatics;
        m_conveyorSubsystem = conveyor;
        m_intakeMotor = new WPI_VictorSPX(Constants.IntakeConstant.kMotor);
        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.setInverted(true);
        m_isSpinning = false;
    }
    public void intake(double power) {
        m_intakeMotor.set(ControlMode.PercentOutput, power);
        m_isSpinning = true;
    }

    public void outtake() {
        m_intakeMotor.set(ControlMode.PercentOutput, -0.5);
        m_isSpinning = true;
    }

    public void stop() {
        m_intakeMotor.set(0);
        m_isSpinning = false;
    }

    public void setIntake(IntakeIndicator upOrDown) {
        switch (upOrDown) {
            case up:
                m_pneumaticSubsystem.intakeUp();
                break;
            case down:
                m_pneumaticSubsystem.intakeDown();
                break;
            default:
                break;
        }
    }

    public IntakeIndicator getIntakeUpOrDown() {
        return m_pneumaticSubsystem.getCurrentIntake();
    }

    public boolean isSpinning() {
        return m_isSpinning;
    }

    public void setOverride(Boolean overrideValue) {
        override = overrideValue;
    }
}
