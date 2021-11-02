/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortID;

public class AirCompressor extends SubsystemBase
{
  private Compressor air_compressor;
  private Solenoid   s1;
  private boolean    Triggered;

  public AirCompressor()
  {
    air_compressor = new Compressor();
    s1             = new Solenoid(PortID.compressor_port.value);
    Triggered      = false;
    air_compressor.start();
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    s1.set(Triggered);
  }

  public void Control_Update()
  {  
    Triggered = !Triggered;  
  }

  public void CompressStop()
  {
    Triggered = false;
    s1.set(false);
  }
}