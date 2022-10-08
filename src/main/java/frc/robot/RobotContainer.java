// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.crypto.ShortBufferException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.DriveByController;
import frc.robot.commands.HoodDown;
import frc.robot.commands.HoodUp;
import frc.robot.commands.KickCommand;
import frc.robot.commands.LiftDown;
import frc.robot.commands.LiftUp;
import frc.robot.commands.PickupCommand;
import frc.robot.commands.ReversePickupCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopAimCommand;
import frc.robot.commands.StopKickCommand;
import frc.robot.commands.LiftStop;
//import frc.robot.commands.StopAimCommand;
import frc.robot.commands.StopPickupCommand;
import frc.robot.commands.StopShootCommand;
import frc.robot.commands.TurretAimCommand;
//import frc.robot.commands.TurretAimCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utilities.JoystickAnalogButton;
import frc.robot.utilities.JoystickAnalogButton.Side;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final XboxController driver = new XboxController(0);
  public Joystick driver2;

  private final Drivetrain m_drive = new Drivetrain();
  private final DriveByController m_driveRobot = new DriveByController(m_drive, driver);
  public JoystickButton intake;
  public JoystickButton outake;
  public JoystickButton climbUp;
  public JoystickButton climbDown;
  public JoystickButton hoodUp;
  public JoystickButton hoodDown;
  public JoystickButton shoot;
  public JoystickButton aim;
  public JoystickButton kick;
  private final Command m_autoCommand = new WaitCommand(20.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drive.setDefaultCommand(m_driveRobot);
    driver2 = new Joystick(1);
    intake = new JoystickButton(driver, Constants.BACK_RIGHT_BUTTON);
    outake = new JoystickButton(driver, Constants.BACK_LEFT_BUTTON);
    shoot = new JoystickButton(driver2, Constants.BACK_BUTTON);
    climbUp = new JoystickButton(driver2, Constants.BUTTON_Y);
    climbDown = new  JoystickButton(driver2, Constants.BUTTON_A);
    hoodUp = new  JoystickButton(driver2, Constants.BUTTON_X);
    hoodDown = new JoystickButton(driver2, Constants.BUTTON_B);
    aim = new JoystickButton(driver2, Constants.BACK_LEFT_BUTTON);
    kick = new JoystickButton(driver2, Constants.BACK_RIGHT_BUTTON);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  /*  new POVButton(m_controller, 0)
        .whenPressed(() -> m_drive.resetOdometry(new Rotation2d(0.0)));
*/
intake.whileHeld(new PickupCommand());
intake.whenReleased(new StopPickupCommand());
outake.whileHeld(new ReversePickupCommand());
outake.whenReleased(new StopPickupCommand());
climbUp.whileHeld(new LiftUp());
climbUp.whenReleased(new LiftStop());
climbDown.whileHeld(new LiftDown());
climbDown.whenReleased(new LiftStop());
hoodUp.whenPressed(new HoodUp());
hoodDown.whenPressed(new HoodDown());
shoot.whileHeld(new ShootCommand());
shoot.whenReleased(new StopShootCommand());
aim.whileHeld(new TurretAimCommand());
kick.whileHeld(new KickCommand());
kick.whenReleased(new StopKickCommand()); 

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
