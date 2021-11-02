// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.TurnSubsystem;

public class ShootCommand extends CommandBase
{
  public static enum ShootMode
  {
    eMega(0),
    eLong(1),
    eMid(2),
    eShort(3),
    eOff(99);

    @SuppressWarnings("MemberName")
    public final int value;
  
    ShootMode(int value) { this.value = value; }
  }

  private ShootSubsystem   my_shoot;
  private TurnSubsystem    my_turn;
  private ControlSubsystem my_control;
  private ShootMode        eMode;
  private double           Speed;
  private double           SpeedLimit;

  public ShootCommand(ShootSubsystem my_shoot, TurnSubsystem my_turn, ControlSubsystem my_control, ShootMode eMode)
  {
    this.my_shoot   = my_shoot;
    this.my_turn    = my_turn;
    this.my_control = my_control;
    this.eMode      = eMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(my_shoot, my_turn, my_control);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if(eMode == null)
      eMode = ShootMode.eOff;
    switch(eMode)
    {
      case eMega:
        Speed      = 0.45f;
        SpeedLimit = 0.5f;
        break;
      case eLong:
        Speed      = 0.44f;
        SpeedLimit = 0.49f;
        break;
      case eMid:
        Speed      = 0.495f;
        SpeedLimit = 0.545f;
        break;
      case eShort:
        Speed      = 0.54f;
        SpeedLimit = 0.59f;
        break;
      default:
        Speed      = 0.0f;
        SpeedLimit = 0.0f;
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    my_shoot.Shoot(Speed);
    if(my_shoot.getShootMotor().getMotorOutputVoltage() < SpeedLimit)
    {
      if(my_shoot.getShootMotor().getMotorOutputVoltage() > SpeedLimit && my_shoot.getShootMotor().getMotorOutputVoltage() != 0.0f)
      {
        my_turn.Spin(0.25f);
        my_control.control(0.4f);
      }
      else if(my_shoot.getShootMotor().getMotorOutputVoltage() > SpeedLimit && my_shoot.getShootMotor().getMotorOutputVoltage() != 0.0f)
      {
        my_shoot.Shoot(Speed * 1.5f);
        my_control.control(0.05f);
        my_turn.SpinStop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return eMode == ShootMode.eOff;
  }
}