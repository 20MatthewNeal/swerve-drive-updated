// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoystick extends CommandBase {

  private SwerveDrive swerveDrive;
  private Joystick joy;
  private Supplier<Double> rotateSpeed;
  private boolean fieldOriented;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter rotateLimiter;

  private ChassisSpeeds chassisSpeeds;
  private SwerveModuleState[] moduleStates;
  private SwerveDriveKinematics swerveKinematics;

  /** Creates a new DriveWithJoystick. */
  public DriveWithJoystick(SwerveDrive swerveDrive, Joystick joy, boolean fieldOriented) {
    this.swerveDrive = swerveDrive;
    this.joy = joy;
    this.fieldOriented = fieldOriented;

    this.xLimiter = new SlewRateLimiter(DriveConstants.MAX_DRIVE_SPEED);
    this.yLimiter = new SlewRateLimiter(DriveConstants.MAX_DRIVE_SPEED);
    this.rotateLimiter = new SlewRateLimiter(0);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveKinematics = new SwerveDriveKinematics(new Translation2d(DriveConstants.DIST_FROM_CENTER, DriveConstants.CENTER_ANGLE));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = joy.getX();
    double ySpeed = joy.getY();
    double rotateSpeed = joy.getDirectionDegrees();

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);

    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(rotateSpeed);
    rotateSpeed = rotateLimiter.calculate(rotateSpeed) * DriveConstants.ROTATE_VELOCITY_CONVERSION;

    if(swerveDrive.getFieldOriented()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, swerveDrive.getRotation2d());      
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
    }

    moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveDrive.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
