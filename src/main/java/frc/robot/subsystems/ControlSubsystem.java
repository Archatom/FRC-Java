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

public class ControlSubsystem extends SubsystemBase
{
  private VictorSPX my_control;

  public ControlSubsystem()
  {
    my_control = new VictorSPX(PortID.control_port.value);
    MotorInit();
  }

  /**
   * Controls the control motor speed
   * @param speed the commanded control output
   */
  public void control(double speed)
  {
    my_control.set(ControlMode.PercentOutput, speed);
  }

  /**
   * set the control speed to 0
   */
  public void controlStop()
  {
    my_control.set(ControlMode.PercentOutput, 0.0f);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  /**
   * Initilize the Control motor settings
   */
  private void MotorInit()
  {
    my_control.configFactoryDefault();
    my_control.setInverted(PortReversed.control_port_reversed.value);
    my_control.configOpenloopRamp(Constants.kMotorRampRate);
    my_control.setNeutralMode(NeutralMode.Brake);
  }
}