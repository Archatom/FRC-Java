// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurnSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.commands.LimeLightCommand.LightMode;

public class AutonomousCommand extends CommandBase
{
  private DriveSubsystem   my_drive;
  private IntakeSubsystem  my_intake;
  private TurnSubsystem    my_turn;
  private ShootSubsystem   my_shoot;
  private ControlSubsystem my_control;
  private LimeLightCommand CamCommand;
  private boolean          finished;
  private Timer            timer;
  
  public AutonomousCommand(DriveSubsystem my_drive, IntakeSubsystem my_intake, TurnSubsystem my_turn, ShootSubsystem my_shoot, LimeLight my_limelight, ControlSubsystem my_control, LimeLightCommand CamCommand)
  {
    this.my_drive     = my_drive;
    this.my_intake    = my_intake;
    this.my_turn      = my_turn;
    this.my_shoot     = my_shoot;
    this.my_control   = my_control;
    this.CamCommand   = CamCommand;
    timer             = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    // 需要加入所有引進AutonomousCommand的子系統
    addRequirements(my_drive, my_intake, my_turn, my_shoot, my_limelight, my_control);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer.reset();
    timer.start();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    //my_intake.intake(0.2f, 0.5f);
    my_control.control(0.4f);                                 // 板機持續開啟
    my_shoot.Shoot(0.45f);                                    // 飛輪持續開啟

    if(timer.get() <= 3.0f)
    {
      CamCommand.SetMode(LightMode.eAim);                     // t=0 瞄準3秒
    }
    else if(timer.get() <= 7.0f)
    {
      my_turn.Spin(0.3f);                                     // t=3 轉盤4秒
    }
    else if(timer.get() <= 9.0f)                              // t=7 往後1秒
    {
      my_drive.tankdrive(0.5f, 0.5f);
    }
    else
    {
      CamCommand.SetMode(LightMode.eOff);
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    my_drive.DriveStop();
    my_turn.SpinStop();
    my_shoot.ShootStop();
    my_intake.intakeStop();
    my_control.controlStop();
    CamCommand.SetMode(LightMode.eOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return finished;
  }
}
