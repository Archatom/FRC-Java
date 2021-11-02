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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.Constants.PortReversed;

public class TurnSubsystem extends SubsystemBase
{
  private VictorSPX up_turn;
  private VictorSPX turn_motor;
  private Encoder   turn_encoder;
  private Boolean   AutoSpin;
  private XboxController my_xbox;

  public TurnSubsystem(XboxController my_xbox)
  {
    AutoSpin     = false;
    up_turn      = new VictorSPX(PortID.up_turn_port.value);
    turn_motor   = new VictorSPX(PortID.turn_motor_port.value);
    turn_encoder = new Encoder(
      Constants.kTurnEncoderPorts[0], Constants.kTurnEncoderPorts[1],
      PortReversed.turn_encoder_reversed.value);
    this.my_xbox = my_xbox;
    MotorInit();
    EncoderInit();
  }

  public void turnMotor(double speed)
  {
    up_turn.set(ControlMode.PercentOutput, speed);
  }

  public void turnStop()
  {
    up_turn.set(ControlMode.PercentOutput, 0.0f);
  }

  /**
   * set the spinner speed
   * @param speed the commanded spinner output
   */
  public void Spin(double speed)
  {
    turn_motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * set the spinner speed to 0
   */
  public void SpinStop()
  {
    turn_motor.set(ControlMode.PercentOutput, 0.0f);
  }

  public void AutoSpinSwitch()
  {
    AutoSpin = !AutoSpin;
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    turnMotor(my_xbox.getRawAxis(0) * -0.2f);
    if(my_xbox.getTriggerAxis(Hand.kRight) > 0) Spin(0.3f);
    if(my_xbox.getTriggerAxis(Hand.kLeft) > 0) Spin(-0.3f);
    if(!AutoSpin) Spin(0.0f);
    SmartDashboard.putNumber("turn encoder", turn_encoder.getDistance());
  }

  /**
   * Initilize the Turn motors settings
   */
  private void MotorInit()
  {
    up_turn.configFactoryDefault();
    turn_motor.configFactoryDefault();
    up_turn.configOpenloopRamp(Constants.kMotorRampRate);
    turn_motor.configOpenloopRamp(Constants.kMotorRampRate);
    up_turn.setInverted(PortReversed.up_turn_port_reversed.value);
    turn_motor.setInverted(PortReversed.turn_motor_port_reversed.value);
    up_turn.setNeutralMode(NeutralMode.Brake);
    turn_motor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Initilize the Turn encoder settings
   */
  private void EncoderInit()
  {
    turn_encoder.setDistancePerPulse(0.005f);
    turn_encoder.reset();
  }
}