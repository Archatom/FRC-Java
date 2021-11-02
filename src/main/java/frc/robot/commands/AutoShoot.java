/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.TurnSubsystem;

public class AutoShoot extends CommandBase
{
  private ControlSubsystem my_control;
  private ShootSubsystem   my_shoot;
  private TurnSubsystem    my_turn;
  private IntakeSubsystem  my_intake;
  private boolean          finished;
  private Timer            timer;

  public AutoShoot(ShootSubsystem my_shoot, ControlSubsystem my_control, TurnSubsystem my_turn, IntakeSubsystem my_intake)
  {
    this.my_control = my_control;
    this.my_shoot   = my_shoot;
    this.my_turn    = my_turn;
    this.my_intake  = my_intake;
    timer           = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(my_shoot, my_control, my_turn, my_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer.reset();
    timer.start();
    finished = false;
    my_control.controlStop();
    my_shoot.ShootStop();
    my_turn.SpinStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    my_control.controlStop();
    my_shoot.ShootStop();
    my_turn.SpinStop();

    my_intake.AutoIntakeSwitch();
    my_turn.AutoSpinSwitch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return finished;
  }
}