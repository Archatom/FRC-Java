/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import java.io.IOException;
// import java.nio.file.Path;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import frc.robot.Constants.TrajrctoryJSON;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.LimeLightCommand;
import frc.robot.commands.LimeLightCommand.LightMode;
import frc.robot.subsystems.AirCompressor;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ControlSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.TurnSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final XboxController   xbox_drive = new XboxController(0);
  private final XboxController   xbox_shoot = new XboxController(1);

  private final LimeLight        my_cam     = new LimeLight();
  private final DriveSubsystem   my_drive   = new DriveSubsystem(xbox_drive);
  private final IntakeSubsystem  my_intake  = new IntakeSubsystem();
  private final AirCompressor    my_air     = new AirCompressor();
  private final TurnSubsystem    my_turn    = new TurnSubsystem(xbox_shoot);
  private final ControlSubsystem my_control = new ControlSubsystem();
  private final ShootSubsystem   my_shoot   = new ShootSubsystem();
  private final ClimbSubsystem   my_climb   = new ClimbSubsystem();

  private final LimeLightCommand CamCommand = new LimeLightCommand(my_cam, my_drive, my_turn);

  private static Timer  timer = new Timer();
  private static double air_elapsed;
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    timer.start();
    air_elapsed = 0.0f;
    my_cam.setDefaultCommand(CamCommand);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   */
  private void configureButtonBindings()
  {
    final POVButton drive_up    = new POVButton(xbox_drive, 0);
    // final POVButton drive_right  = new POVButton(xbox_drive, 90);
    final POVButton drive_down  = new POVButton(xbox_drive, 180);
    final POVButton drive_left  = new POVButton(xbox_drive, 270);

    drive_up.whenPressed(()->my_climb.Elevate(0.5f));
    drive_up.whenReleased(()->my_climb.Elevate(0.0f));
    drive_down.whenPressed(()->my_climb.Elevate(-0.6f));
    drive_down.whenReleased(()->my_climb.Elevate(0.0f));

    new JoystickButton(xbox_drive, XboxController.Button.kStart.value).whenPressed(()->my_climb.Climb(-0.3f));
    new JoystickButton(xbox_drive, XboxController.Button.kStart.value).whenReleased(()->my_climb.Climb(0.0f));
    drive_left.whenPressed(()->my_climb.Climb(0.3f));
    drive_left.whenReleased(()->my_climb.Climb(0.0f));

    // Climb&Elevate stop
    new JoystickButton(xbox_drive, XboxController.Button.kB.value).whenPressed(()->my_climb.ClimbStop());
    new JoystickButton(xbox_drive, XboxController.Button.kBack.value).whenPressed(()->my_climb.ElevateStop());

    // Intake observe
    new JoystickButton(xbox_drive, XboxController.Button.kBumperRight.value).whenPressed(()->{
      my_intake.intake(0.2f, 0.65f);
      my_turn.Spin(0.6f);
    });
    new JoystickButton(xbox_drive, XboxController.Button.kBumperRight.value).whenReleased(()->{
      my_intake.intakeStop();
      my_turn.SpinStop();
    });

    // Intake reverse
    new JoystickButton(xbox_drive, XboxController.Button.kBumperLeft.value).whenPressed(()->{
      my_intake.intake(-0.2f, -0.65f);
      my_turn.Spin(-0.6f);
    });
    new JoystickButton(xbox_drive, XboxController.Button.kBumperLeft.value).whenReleased(()->{
      my_intake.intakeStop();
      my_turn.SpinStop();
    });

    // Air Compressor
    new JoystickButton(xbox_drive, XboxController.Button.kA.value).whileHeld(()->{
      if(timer.get() - air_elapsed >= 0.5f)
      {
        air_elapsed = timer.get();
        my_air.Control_Update();
      }
    });



    /*###################*/
    /*#Second Controller#*/
    /*###################*/

    final POVButton shoot_up    = new POVButton(xbox_shoot, 0);
    final POVButton shoot_right = new POVButton(xbox_shoot, 90);
    final POVButton shoot_down  = new POVButton(xbox_shoot, 180);
    final POVButton shoot_left  = new POVButton(xbox_shoot, 270);

    //shoot
    shoot_up.whenPressed(()->my_shoot.Shoot(0.4f));
    shoot_right.whenPressed(()->my_shoot.Shoot(0.5f));
    shoot_down.whenPressed(()->my_shoot.Shoot(0.55f));
    shoot_left.whenPressed(()->my_shoot.Shoot(0.6f));
    
    shoot_up.whenReleased(()->my_shoot.Shoot(0.0f));
    shoot_right.whenReleased(()->my_shoot.Shoot(0.0f));
    shoot_down.whenReleased(()->my_shoot.Shoot(0.0f));
    shoot_left.whenReleased(()->my_shoot.Shoot(0.0f));

    // Control obverse
    new JoystickButton(xbox_shoot, XboxController.Button.kX.value).whenPressed(()->my_control.control(0.4f));
    new JoystickButton(xbox_shoot, XboxController.Button.kX.value).whenReleased(()->my_control.control(-0.1f));
    // Control reserve
    new JoystickButton(xbox_shoot, XboxController.Button.kY.value).whenPressed(()->my_control.control(-0.2f));
    new JoystickButton(xbox_shoot, XboxController.Button.kY.value).whenReleased(()->my_control.control(-0.1f));

    // Spin
    new JoystickButton(xbox_shoot, XboxController.Button.kA.value).whenPressed(()->my_turn.Spin(0.3f));
    new JoystickButton(xbox_shoot, XboxController.Button.kA.value).whenReleased(()->my_turn.SpinStop());
    // Reverse Spin
    new JoystickButton(xbox_shoot, XboxController.Button.kB.value).whenPressed(()->my_turn.Spin(-0.3f));
    new JoystickButton(xbox_shoot, XboxController.Button.kB.value).whenReleased(()->my_turn.SpinStop());

    // Intake balls
    new JoystickButton(xbox_shoot, XboxController.Button.kBumperRight.value).whenPressed(()->{
      my_intake.intake(0.15f, 0.65f);
      my_turn.Spin(0.3f);
    });
    new JoystickButton(xbox_shoot, XboxController.Button.kBumperRight.value).whenReleased(()->{
      my_intake.intakeStop();
      my_turn.SpinStop();
    });
    // Intake Reverse
    new JoystickButton(xbox_shoot, XboxController.Button.kBumperLeft.value).whenPressed(()->{
      my_intake.intake(-0.15f, -0.65f);
      my_turn.Spin(-0.3f);
    });
    new JoystickButton(xbox_shoot, XboxController.Button.kBumperLeft.value).whenReleased(()->{
      my_intake.intakeStop();
      my_turn.SpinStop();
    });

    // LimeLight auto aim
    new JoystickButton(xbox_shoot, XboxController.Button.kStart.value).whileHeld(()->CamCommand.SetMode(LightMode.eAim));
    new JoystickButton(xbox_shoot, XboxController.Button.kStart.value).whenReleased(()->CamCommand.SetMode(LightMode.eOff));
  }
                                                                                        

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    AutonomousCommand AutoCommand = new AutonomousCommand(my_drive, my_intake, my_turn, my_shoot, my_cam, my_control, CamCommand);

    return AutoCommand;
  }
}