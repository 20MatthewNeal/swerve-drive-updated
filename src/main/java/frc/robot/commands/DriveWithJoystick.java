// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoystick extends CommandBase {

  private SwerveDrive swerveDrive;
  private Joystick joy;
  private Supplier<Double> rotateSpeed;
  private Supplier<Boolean> fieldOriented;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotateLimiter;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(SwerveDrive swerveDrive, Joystick joy, Supplier<Boolean> fieldOriented) {
    this.swerveDrive = swerveDrive;
    this.joy = joy;
    this.fieldOriented = fieldOriented;

    this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_SPEED)
    this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_SPEED);
    this.rotateLimiter = new SlewRateLimiter(0);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = joy.getX();
    double ySpeed = joy.getY();
    double rotateSpeed = joy.getDirectionDegrees();

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
