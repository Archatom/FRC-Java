/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants
{

	/**
	 * All the IDs of the ports
	 */
	public static enum PortID
	{
		left_front_port(5),
		left_back_port(6),
		right_front_port(8),
		right_back_port(7),
		intake_up_port(2),
		intake_down_port(3),
		up_turn_port(5),
		turn_motor_port(1),
		control_port(4),
		shoot_port(11),  	 // Falcon 500
		climb_port(9),
		elevator_port(14),
		compressor_port(7),  // air compressor
		pigeon_port(9);		 // TalonSRX

		@SuppressWarnings("MemberName")
		public final int value;

		PortID(int value) { this.value = value; }
	}

	/**
	 * Define whether any of these ports are reversed
	 */
	public static enum PortReversed
	{
		kGyroReversed(false),
		left_motors_reversed(true),
		right_motors_reversed(true),
		up_turn_port_reversed(true),
		turn_motor_port_reversed(false),
		control_port_reversed(false),
		left_encoder_reversed(false),
		right_encoder_reversed(false),
		intake_up_reversed(true),
		intake_down_reversed(true),
		shoot_reversed(true),
		climb_reversed(false),
		elevator_reverseed(false),
		turn_encoder_reversed(true);

		@SuppressWarnings("MemberName")
		public final boolean value;

		PortReversed(boolean value) { this.value = value; }
	}
	
    public static final double ksVolts						= 0.147f;
    public static final double kvVoltSecondsPerMeter 		= 2.82f;
    public static final double kaVoltSecondsSquaredPerMeter = 0.524f;

	public static final double kGearReduction    = 10.75f;  // The gear ratio
    public static final double kTrackwidthMeters = 0.65706f;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

	public static final double kMaxSpeedMetersPerSecond				  = 2.5f;
	public static final double kMaxAccelerationMetersPerSecondSquared = 1.0f;
	
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB	= 2.0f;
	public static final double kRamseteZeta = 0.7f;

	public static final double Kp = 0.443f;
	public static final double Ki = 0.0f;
	public static final double Kd = 0.0f;

	public static final int[] kTurnEncoderPorts  = {0, 1};

	public static final int    kEncoderCPR 				= 360;
    public static final double kWheelDiameterMeters 	= 0.1524f;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / 60.0f;
													   // Assumes the encoders are directly mounted on the wheel shafts

	public static final double kMotorRampRate 	= 0.25f;
	public static final double kLimeLightAdjust = 0.01f;
	public static final int    ktimeoutMs 		= 30;

	public static enum TrajrctoryJSON
	{
		TestPath("paths/test.wpilib.json"),
		BlueBallPath("paths/BlueBallPath.wpilib.json"),
		BlueSimplePath("paths/BlueSimplePath.wpilib.json"),
		RedBallPath("paths/RedBallPath.wpilib.json"),
		RedSimplePath("paths/RedSimplePath.wpilib.json"),
		StalomPath("paths/StalomPath.wpilib.json"),
		BarrelPath("paths/BarrelRacing.wpilib.json"),
		BouncePath("paths/Bounce.wpilib.json");

		@SuppressWarnings("MemberName")
		public final String value;

		TrajrctoryJSON(String value) { this.value = value; }
	}
   
}