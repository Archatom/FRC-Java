/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.Constants.PortReversed;

public class IntakeSubsystem extends SubsystemBase
{
  private VictorSPX intake_up;
  private VictorSPX intake_down;
  private Boolean   AutoIntake;

  public IntakeSubsystem()
  {
    intake_up   = new VictorSPX(PortID.intake_up_port.value);
    intake_down = new VictorSPX(PortID.intake_down_port.value);
    MotorInit();
    AutoIntake = false;
  }

  /**
   * Controls the intake motor speed
   *
   * @param speed the commanded intake output
   */
  public void intake(double up_speed, double down_speed)    //上面&下面的intake
  {
    intake_up.set(ControlMode.PercentOutput, up_speed);
    intake_down.set(ControlMode.PercentOutput, down_speed);
  }

  /**
   * set intake speed to 0
   */
  public void intakeStop()
  {
    intake_up.set(ControlMode.PercentOutput, 0.0f);
    intake_down.set(ControlMode.PercentOutput, 0.0f);
  }

  public void AutoIntakeSwitch()
  {
    AutoIntake = !AutoIntake;
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    if(AutoIntake)
      intake(-0.2f, -0.5f);
  }

  /**
   * Initilize the Intake motors settings
   */
  private void MotorInit()
  {
    intake_up.configFactoryDefault();
    intake_down.configFactoryDefault();
    intake_up.configOpenloopRamp(Constants.kMotorRampRate);
    intake_down.configOpenloopRamp(Constants.kMotorRampRate);
    intake_up.setInverted(PortReversed.intake_up_reversed.value);
    intake_down.setInverted(PortReversed.intake_down_reversed.value);
    intake_up.setNeutralMode(NeutralMode.Brake);
    intake_down.setNeutralMode(NeutralMode.Brake);
  }
}