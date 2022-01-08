/*----------------------------------------------------------------------------*/
/* Copyright (c) 2021 Team 5827, Code Purple. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX motor1 = new WPI_TalonFX(Constants.ShooterConstants.kShooterMotor1);
    private final WPI_TalonFX motor2 = new WPI_TalonFX(Constants.ShooterConstants.kShooterMotor2);
    private double m_RPMBottomDesired;
    private double m_RPMBottomActual;
    private double m_RPMTopDesired;
    private double m_RPMTopActual;
    private double m_RPMBottomDesiredLast = 0.0;
    private double m_RPMTopDesiredLast = 0.0;
    private NetworkTableEntry m_RPMBottomDesiredTableEntry;
    private NetworkTableEntry m_RPMBottomActualTableEntry;
    private NetworkTableEntry m_RPMTopDesiredTableEntry;
    private NetworkTableEntry m_RPMTopActualTableEntry;
    private NetworkTableEntry m_shooterP;
    private NetworkTableEntry m_shooterI;
    private NetworkTableEntry m_shooterD;
    private NetworkTableEntry m_shooterF;

    private NetworkTableEntry m_RPMChooser;

    private enum RPMOptions { ZeroRPM, OnekRPM, One5kRPM, TwokRPM, Two5kRPM, ThreekRPM, Three5kRPM,
        FourkRPM, Four5kRPM, FivekRPM, Five5kRPM, SixkRPM }

    private SendableChooser<RPMOptions> m_RPMSendableChooser;

    public ShooterSubsystem() {

        m_RPMBottomDesiredTableEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter Bottom RPM (desired)");
        m_RPMTopDesiredTableEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter Top RPM (desired)");
        m_RPMBottomActualTableEntry =  NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter Bottom RPM (actual)");
        m_RPMTopActualTableEntry =  NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter Top RPM (actual)");

        m_shooterP = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter kP");
        m_shooterI = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter kI");
        m_shooterD = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter kD");

        // make sure to calculate for typical (expected) speeds
        // max output resulted in 22320 ticks / 100ms, however rated is 6380 RPM (21777)
        // F-gain = (100% * 1023) / 21777 (theoretical)
        // or -- F-gain = (75% * 1023) / 16786 (measured value)
        // Output of Talon PIDF controller uses 1023 as full output
        m_shooterF = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Shooter kF");

        m_RPMSendableChooser = new SendableChooser<>();
        m_RPMSendableChooser.setDefaultOption("0 RPM", RPMOptions.ZeroRPM);
        m_RPMSendableChooser.addOption("1000 RPM", RPMOptions.OnekRPM);
        m_RPMSendableChooser.addOption("1500 RPM", RPMOptions.One5kRPM);
        m_RPMSendableChooser.addOption("2000 RPM", RPMOptions.TwokRPM);
        m_RPMSendableChooser.addOption("2500 RPM", RPMOptions.Two5kRPM);
        m_RPMSendableChooser.addOption("3000 RPM", RPMOptions.ThreekRPM);
        m_RPMSendableChooser.addOption("3500 RPM", RPMOptions.Three5kRPM);
        m_RPMSendableChooser.addOption("4000 RPM", RPMOptions.FourkRPM);
        m_RPMSendableChooser.addOption("4500 RPM", RPMOptions.Four5kRPM);
        m_RPMSendableChooser.addOption("5000 RPM", RPMOptions.FivekRPM);
        m_RPMSendableChooser.addOption("5500 RPM", RPMOptions.Five5kRPM);
        m_RPMSendableChooser.addOption("6000 RPM", RPMOptions.SixkRPM);
        
   
        SmartDashboard.putData("Shooter RPM", m_RPMSendableChooser);
        m_RPMChooser = NetworkTableInstance.getDefault().getTable("SmartDashboard/Shooter RPM").getEntry("active");

        m_RPMChooser.addListener(event -> {
                RPMOptions value = m_RPMSendableChooser.getSelected();
                switch (value) {
                    case ZeroRPM:
                        setSpeedInRPM(0);
                        break;
                    case OnekRPM:
                        setSpeedInRPM(1000);
                        break;
                    case One5kRPM:
                        setSpeedInRPM(1500);
                        break;
                    case TwokRPM:
                        setSpeedInRPM(2000);
                        break;
                    case Two5kRPM:
                        setSpeedInRPM(2500);
                        break;
                    case ThreekRPM:
                        setSpeedInRPM(3000);
                        break;
                    case Three5kRPM:
                        setSpeedInRPM(3500);
                        break;
                    case FourkRPM:
                        setSpeedInRPM(4000);
                        break;
                    case Four5kRPM:
                        setSpeedInRPM(4500);
                        break;
                    case FivekRPM:
                        setSpeedInRPM(5000);
                        break;
                    case Five5kRPM:
                        setSpeedInRPM(5500);
                        break;
                    case SixkRPM:
                        setSpeedInRPM(6500);
                        break;
                    default:
                        setSpeedInRPM(0);
                }
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate | EntryListenerFlags.kLocal);

    
        // 0 RPM to start in case there is a leftover value in dashboard
        m_RPMBottomDesiredTableEntry.setDouble(0.0);
        m_RPMTopDesiredTableEntry.setDouble(0.0);

        m_shooterP.setDouble(0.25);
        m_shooterI.setDouble(0.00);
        m_shooterD.setDouble(5.00);
        m_shooterF.setDouble(0.055);

        // resets Talons to default values, in case there were any configured in the motor controller
        motor1.configFactoryDefault();
        motor2.configFactoryDefault();

        motor1.configOpenloopRamp(0.2);
        motor2.configOpenloopRamp(0.2);

        motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        motor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        motor1.configClosedLoopPeakOutput(0, 1.0);
        motor2.configClosedLoopPeakOutput(0, 1.0);

        motor1.config_kP(0, m_shooterP.getDouble(0.0));
        motor2.config_kP(0, m_shooterP.getDouble(0.0));

        motor1.config_kI(0, m_shooterI.getDouble(0.0));
        motor2.config_kI(0, m_shooterI.getDouble(0.0));

        motor1.config_kD(0, m_shooterD.getDouble(0.0));
        motor2.config_kD(0, m_shooterD.getDouble(0.0));

        motor1.config_kF(0, m_shooterF.getDouble(0.0));
        motor2.config_kF(0, m_shooterF.getDouble(0.0));

        motor1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));
        motor2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));

        motor1.setNeutralMode(NeutralMode.Coast);
        motor2.setNeutralMode(NeutralMode.Coast);

        // As looking towards the front (shaft) side of the Falcon 500
        motor1.setInverted(TalonFXInvertType.Clockwise);

        //motor2.follow(motor1);
        motor2.setInverted(TalonFXInvertType.CounterClockwise);
    }

    @Override
    public void periodic() {
        getSpeedInRPM();

        m_RPMBottomDesired = m_RPMBottomDesiredTableEntry.getDouble(0.0);
        m_RPMTopDesired = m_RPMTopDesiredTableEntry.getDouble(0.0);
        
        if ((m_RPMBottomDesiredLast != m_RPMBottomDesired) || (m_RPMTopDesiredLast != m_RPMTopDesired)) {
            m_RPMBottomDesiredLast = m_RPMBottomDesired;
            m_RPMTopDesiredLast = m_RPMTopDesired;

            motor1.config_kP(0, m_shooterP.getDouble(0.0));
            motor2.config_kP(0, m_shooterP.getDouble(0.0));

            motor1.config_kI(0, m_shooterI.getDouble(0.0));
            motor2.config_kI(0, m_shooterI.getDouble(0.0));

            motor1.config_kD(0, m_shooterD.getDouble(0.0));
            motor2.config_kD(0, m_shooterD.getDouble(0.0));

            motor1.config_kF(0, m_shooterF.getDouble(0.0));
            motor2.config_kF(0, m_shooterF.getDouble(0.0));
        }

        setSpeedInRPMInternal(m_RPMBottomDesiredLast, m_RPMTopDesiredLast);
    }

    public void turnMotorOff() {
        motor1.set(0);
        motor2.set(0);
    }

    public void turnMotorOn() {
        motor1.set(0.1);
        motor2.set(0.1);
    }

    public double getSpeedInRPM() {
        m_RPMBottomActual = motor1.getSelectedSensorVelocity() / 2048.0 * 600;
        m_RPMTopActual = motor2.getSelectedSensorVelocity() / 2048.0 * 600;
        m_RPMBottomActualTableEntry.setDouble(m_RPMBottomActual);
        m_RPMTopActualTableEntry.setDouble(m_RPMTopActual);
        return m_RPMBottomActual;
    }

    public void setSpeedInRPM(double RPMSpeed) {
        m_RPMBottomDesired = m_RPMTopDesired = RPMSpeed;
        m_RPMBottomDesiredTableEntry.setDouble(RPMSpeed);
        m_RPMTopDesiredTableEntry.setDouble(RPMSpeed);
        setSpeedInRPMInternal(RPMSpeed, RPMSpeed);
    }

    public void setSpeedInRPM(double RPMSpeedBottom, double RPMSpeedTop) {
        m_RPMBottomDesired = RPMSpeedBottom;
        m_RPMTopDesired = RPMSpeedTop;
        m_RPMBottomDesiredTableEntry.setDouble(RPMSpeedBottom);
        m_RPMTopDesiredTableEntry.setDouble(RPMSpeedTop);
        setSpeedInRPMInternal(RPMSpeedBottom, RPMSpeedTop);
    }

    private void setSpeedInRPMInternal(double RPMBottomSpeed, double RPMTopSpeed) {
        // Falcon 500 encoder has 2048 ticks per revolution.  Units are per 100ms.
        motor1.set(TalonFXControlMode.Velocity, (RPMBottomSpeed * 2048) / 600.0);
        motor2.set(TalonFXControlMode.Velocity, (RPMTopSpeed * 2048) / 600.0);
    }

}
