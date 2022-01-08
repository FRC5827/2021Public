/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.Constants.GearIndicator;
import frc.robot.Constants.IntakeIndicator;

public class PneumaticSubsystem extends SubsystemBase {
  /**
   * Creates a new PneumaticSubsystem.
   */

  private GearIndicator m_gear;
  private IntakeIndicator m_intake;
  
  private DoubleSolenoid shifter = new DoubleSolenoid(PneumaticConstants.PCM_ID,
    PneumaticConstants.SHIFTER_FORWARD_CHANNEL, PneumaticConstants.SHIFTER_BACKWARD_CHANNEL);

  private DoubleSolenoid intake = new DoubleSolenoid(PneumaticConstants.PCM_ID,
    PneumaticConstants.INTAKE_FORWARD_CHANNEL, PneumaticConstants.INTAKE_BACKWARD_CHANNEL);
  

  public PneumaticSubsystem() {
    shifter.set(DoubleSolenoid.Value.kForward);
    m_gear = GearIndicator.low;

    intake.set(DoubleSolenoid.Value.kForward);
    m_intake = IntakeIndicator.up;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shiftUp() {
    shifter.set(DoubleSolenoid.Value.kReverse);
    m_gear = GearIndicator.high;
  }

  public void shiftDown() {
    shifter.set(DoubleSolenoid.Value.kForward);
    m_gear = GearIndicator.low;
  }

  public GearIndicator getCurrentGear() {
    return m_gear;
  }


  public void intakeUp() {
    intake.set(DoubleSolenoid.Value.kReverse);
    m_intake = IntakeIndicator.up;
  }

  public void intakeDown() {
    intake.set(DoubleSolenoid.Value.kForward);
    m_intake = IntakeIndicator.down;
  }

  public IntakeIndicator getCurrentIntake() {
    return m_intake;
  }


}
