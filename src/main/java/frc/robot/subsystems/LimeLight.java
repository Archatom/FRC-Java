/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase
{
  public final double Kp_aim       =  0.03f;
  public final double Kp_distance  = -0.5f;
  public final double desired_area =  10.0f;
  public final double max_speed    =  0.4f;
  private NetworkTable      table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  public LimeLight()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx    = table.getEntry("tx");
    ty    = table.getEntry("ty");
    ta    = table.getEntry("ta");
    tv    = table.getEntry("tv");
  }

  /**
   * Get the current X value
   * @return the X value
   */
  public double GetX()
  {  return tx.getDouble(0.0f);  }

  /**
   * Get the current Y value
   * @return the Y value
   */
  public double GetY()
  {  return ty.getDouble(0.0f);  }

  /**
   * Get the current area of target
   * @return the area of target
   */
  public double GetArea()
  {  return ta.getDouble(0.0f);  }

  /**
   * get the V value (if the target is found)
   * @return the V value
   */
  public double GetV()
  {  return tv.getDouble(0.0f);  }

  @Override
  public void periodic()
  {

  }
}
