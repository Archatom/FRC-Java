/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.TurnSubsystem;

public class LimeLightCommand extends CommandBase
{
  /**
   * LimeLight control mode
   */
  public enum LightMode
  {
    eOff(0),
    eSeek(1),
    eRange(2),
    eAim(3),
    eTest(69),
    eError(99);

    @SuppressWarnings("MemberName")
		public final int value;

		LightMode(int value) { this.value = value; }
  };

  private DriveSubsystem my_drive;
  private TurnSubsystem  my_turn;
  private LimeLight      limelight;
  private double         x;
  private double         y;
  private double         area;
  private double         range;
  private double         Value;
  private double         turn;
  private LightMode      eMode;

  public LimeLightCommand(LimeLight limelight, DriveSubsystem my_drive, TurnSubsystem my_turn)
  {
    this.limelight = limelight;
    this.my_drive  = my_drive;
    this.my_turn = my_turn;
    eMode = LightMode.eOff;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, my_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    LimeLightUpdate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    LimeLightUpdate();
    switch(eMode)
    {
      case eOff:
        // nothing to be done.
        break;
  

      case eRange:
        if(Value == 0.0f)  // not found.
          my_drive.ArcadeDrive(-0.4f, 0.0f);
        else
        {
          if((limelight.desired_area - area < 2) && (limelight.desired_area - area > -2))
            my_drive.ArcadeDrive(0.0f, 0.0f);
          else if(limelight.desired_area - area > 2)
            my_drive.ArcadeDrive(((range < limelight.max_speed)? limelight.max_speed : range)*1.4f, turn);
          else
            my_drive.ArcadeDrive((range > -limelight.max_speed)? -limelight.max_speed*1.4f : range*0.75f, turn);
        }
        break;


      case eSeek:
        if(Value == 0.0f)  // not found.
          my_drive.ArcadeDrive(-0.1f, 0.4f);
        else
          my_drive.ArcadeDrive(-0.3f, (x<1 && x>-1)? 0.0f : turn);
        break;
  
      // Automatically aims the score frame.
      case eAim:
        if((x < 0.1f) && (x > -0.1f) && (x != 0.0f))
          my_turn.turnStop();
        else
          my_turn.turnMotor(-x * Constants.kLimeLightAdjust);
        break;

      // new modes are tested here.
      case eTest:
        if(x<1 && x>-1)
          my_drive.tankdrive(0.1f, 0.1f);
        else
          my_drive.tankdrive(x*Constants.kLimeLightAdjust, -x*Constants.kLimeLightAdjust);
        break;


      default: // unexpected.
        eMode = LightMode.eError;
        my_drive.DriveStop();
        break;
    }
    SmartDashboardUpdate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    SmartDashboard.putBoolean("LimeLight Unexpected", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {  return eMode == LightMode.eError;  }

  /**
   *  Update the LimeLight Values
   */
  private void LimeLightUpdate()
  {
    x     = limelight.GetX();
    y     = limelight.GetY();
    area  = limelight.GetArea();
    Value = limelight.GetV();
    turn  = limelight.Kp_aim * x;
    range = limelight.Kp_distance * (limelight.desired_area - area);
  }

  /**
   *  Update the current performance of LimeLight on the SmartDashboard
   */
  private void SmartDashboardUpdate()
  {
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightValue", Value);
    SmartDashboard.putString("LimeLight Mode", eMode.toString());
  }

  /**
   * Set the LimeLight mode
   * @param Mode LimeLight mode to be set to
   */
  public void SetMode(LightMode Mode)
  {
    eMode = Mode;
  }

}