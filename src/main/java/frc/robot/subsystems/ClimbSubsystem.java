// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.Constants.PortReversed;

public class ClimbSubsystem extends SubsystemBase
{
  private CANSparkMax climb_motor;
  private VictorSPX   elevator;

  public ClimbSubsystem()
  {
    climb_motor = new CANSparkMax(PortID.climb_port.value, MotorType.kBrushless);
    elevator    = new VictorSPX(PortID.elevator_port.value);
    MotorInit();
  }

  public void Elevate(double speed)
  {
    elevator.set(ControlMode.PercentOutput, speed);
  }

  public void ElevateStop()
  {
    elevator.set(ControlMode.PercentOutput, 0.0f);
  }

  public void Climb(double speed)
  {
    climb_motor.set(speed);
  }

  public void ClimbStop()
  {
    climb_motor.set(0.0f);
  }

  public void MotorInit()
  {
    climb_motor.restoreFactoryDefaults();
    climb_motor.setOpenLoopRampRate(Constants.kMotorRampRate);
    climb_motor.setInverted(PortReversed.climb_reversed.value);
    elevator.configFactoryDefault();
    elevator.configOpenloopRamp(Constants.kMotorRampRate);
    elevator.setInverted(PortReversed.elevator_reverseed.value);
    elevator.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
