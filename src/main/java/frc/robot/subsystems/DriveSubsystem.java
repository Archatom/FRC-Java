/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortID;
import frc.robot.Constants.PortReversed;

public class DriveSubsystem extends SubsystemBase
{
  private final CANSparkMax left_front  = new CANSparkMax(PortID.left_front_port.value, MotorType.kBrushless);
  private final CANSparkMax left_back   = new CANSparkMax(PortID.left_back_port.value, MotorType.kBrushless);
  private final CANSparkMax right_front = new CANSparkMax(PortID.right_front_port.value, MotorType.kBrushless);
  private final CANSparkMax right_back  = new CANSparkMax(PortID.right_back_port.value, MotorType.kBrushless);

  private final SpeedControllerGroup leftMotors  = new SpeedControllerGroup(left_front, left_back); 
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(right_back, right_front);
  private final DifferentialDrive    mydrive     = new DifferentialDrive(leftMotors, rightMotors);

  private final PIDController leftPIDController  = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);
  private final PIDController rightPIDController = new PIDController(Constants.Kp, Constants.Ki, Constants.Kd);

  private final PigeonIMU myPigeon = new PigeonIMU(new TalonSRX(PortID.pigeon_port.value));
  
  //private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private DifferentialDriveOdometry m_odometry;
  private Pose2d Pose;

  private XboxController my_xbox;

  public DriveSubsystem(XboxController my_xbox)
  {
    this.my_xbox = my_xbox;

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    Pose       = new Pose2d();

    MotorInit();
    EncoderInit();

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    Pose = m_odometry.update(Rotation2d.fromDegrees(getHeading()),
              getLeftEncoder().getPositionConversionFactor(), getRightEncoder().getPositionConversionFactor());

    double speed = my_xbox.getRawAxis(1) * 0.8f * 0.75f;
    double turn  = my_xbox.getRawAxis(4) * 0.38f;
      
    if(my_xbox.getStartButtonPressed())
    {
      SmartDashboard.putBoolean("Got it", true);
      speed *= 1.2f;
    }
    
    double right_speed = speed+turn;
    double left_speed  = speed-turn;

    mydrive.tankDrive(left_speed, right_speed);

    SmartDashboardUpdate();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose()
  {
    return Pose;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(
      left_front.getEncoder().getVelocityConversionFactor(),
      right_front.getEncoder().getVelocityConversionFactor()
    );
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry()
  {
    resetEncoder();
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoder()
  {
    getLeftEncoder().setPosition(0.0f);
    getRightEncoder().setPosition(0.0f);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void ArcadeDrive(double fwd, double rot)
  {
    mydrive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive speed
   *
   * @param left_speed the commanded left output
   * @param right_speed the commanded right output
   */
  public void tankdrive(double left_speed, double right_speed)
  {
    mydrive.tankDrive(left_speed, right_speed);
  }

  /**
   * Update the Smart Dashboard values
   */
  private void SmartDashboardUpdate()
  {
    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("left wheel speed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("right wheel speed", getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("right Encoder", getRightEncoder().getPositionConversionFactor());
    SmartDashboard.putNumber("left Encoder", getLeftEncoder().getPositionConversionFactor());
  }

  /**
   * Initilize the CanEncoders settings
   */
  private void EncoderInit()
  {
    getLeftEncoder().setPositionConversionFactor(Constants.kEncoderDistancePerPulse * 60.0f / Constants.kGearReduction);
    getRightEncoder().setPositionConversionFactor(Constants.kEncoderDistancePerPulse * 60.0f / Constants.kGearReduction);
    getLeftEncoder().setVelocityConversionFactor(Constants.kEncoderDistancePerPulse / Constants.kGearReduction);
    getRightEncoder().setVelocityConversionFactor(Constants.kEncoderDistancePerPulse / Constants.kGearReduction);
    resetEncoder();
  }

  /**
   * Initilize the CANSparkMax motors settings
   */
  private void MotorInit()
  {
    left_front.restoreFactoryDefaults();
    left_back.restoreFactoryDefaults();
    right_front.restoreFactoryDefaults();
    right_back.restoreFactoryDefaults();

    left_front.setOpenLoopRampRate(Constants.kMotorRampRate);
    left_back.setOpenLoopRampRate(Constants.kMotorRampRate);
    right_front.setOpenLoopRampRate(Constants.kMotorRampRate);
    right_back.setOpenLoopRampRate(Constants.kMotorRampRate);

    left_back.follow(left_front);
    right_back.follow(right_front);

    leftMotors.setInverted(PortReversed.left_motors_reversed.value);
    rightMotors.setInverted(PortReversed.right_motors_reversed.value);

    left_front.setIdleMode(IdleMode.kCoast);
    left_back.setIdleMode(IdleMode.kCoast);
    right_front.setIdleMode(IdleMode.kCoast);
    right_back.setIdleMode(IdleMode.kCoast);
  }

  public void DriveStop()
  {
    tankdrive(0.0f, 0.0f);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts)
  {
    leftMotors.setVoltage(leftVolts / 12.0f);
    rightMotors.setVoltage(rightVolts / 12.0f);
    mydrive.feed();
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder()
  {
    return left_front.getEncoder();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder()
  {
    return right_front.getEncoder();
  }

  /**
   * Gets the left PID controller
   * 
   * @return the left PID controller
   */
  public PIDController getLeftPIDcontroller()
  {
    return leftPIDController;
  }

  /**
   * Gets the right PID controller
   * 
   * @return the right PID controller
   */
  public PIDController getRightPIDcontroller()
  {
    return rightPIDController;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput)
  {
    mydrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading()
  {
    myPigeon.setYaw(0, Constants.ktimeoutMs);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading()
  {
    return Math.IEEEremainder(getYaw(), 360) * (PortReversed.kGyroReversed.value? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate()
  {
    return getZrate() * (PortReversed.kGyroReversed.value? -1.0f : 1.0f);
  }  

  public double getYaw()
  {
    double[] angles = new double[3];
    myPigeon.getYawPitchRoll(angles);
    return angles[0];
  }  

  public double getZrate()
  {
    double[] rates = new double[3];
    myPigeon.getRawGyro(rates);
    return rates[2];
  } 
  
  public PigeonIMU getpigeon()
  {
    return myPigeon;
  }
}