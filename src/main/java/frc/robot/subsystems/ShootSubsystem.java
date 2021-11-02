/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.Constants.PortReversed;

public class ShootSubsystem extends SubsystemBase
{
  private TalonFX ShootMotor;
  private boolean AutoShoot;
  
  public ShootSubsystem()
  {
    ShootMotor = new TalonFX(PortID.shoot_port.value);
    MotorInit();

    AutoShoot = false;
  }

  /**
   * Control the shoot motors' speed
   * @param speed the command shoot motors outputs
   */
  public void Shoot(double speed)
  {
    ShootMotor.set(ControlMode.PercentOutput, speed);
  }

  /**
   *  Set the shoot motors outputs to 0
   */
  public void ShootStop()
  {
    ShootMotor.set(ControlMode.PercentOutput, 0.0f);
  }

  public TalonFX getShootMotor()
  {
    return ShootMotor;
  }

  @Override
  public void periodic()
  {
    if(AutoShoot)
      Shoot(0.45f);
  }

  /**
   * Initilize the Shoot motor settings
   */
  private void MotorInit()
  {
    ShootMotor.configFactoryDefault();
    ShootMotor.configOpenloopRamp(Constants.kMotorRampRate);
    ShootMotor.setInverted(PortReversed.shoot_reversed.value);
    ShootMotor.setNeutralMode(NeutralMode.Brake);
  }
}