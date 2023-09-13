// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.SetAngle;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private SwerveDrive swerve;
  private SwerveModule swervemod;
  private Sendable swerveSend;

  private Joystick joy;
  private DriveWithJoystick driveWithJoystick;
  private JoystickButton setAngleButton;
  private SetAngle setAngle;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerve = new SwerveDrive();
    joy = new Joystick(DriveConstants.DRIVE_JOYSTICK_ID);
    driveWithJoystick = new DriveWithJoystick(swerve, joy, swerve.getFieldOriented());
    setAngleButton = new JoystickButton(joy, 4);

    swerve.setDefaultCommand(driveWithJoystick);

    setAngle = new SetAngle(swervemod, swerve);
    
    SmartDashboard.putData(swerve);
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    setAngleButton.onTrue(setAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
