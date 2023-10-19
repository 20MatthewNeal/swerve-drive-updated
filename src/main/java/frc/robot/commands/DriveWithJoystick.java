// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoystick extends CommandBase {

  private SwerveDrive swerveDrive;
  private Joystick joy;
  private Supplier<Double> rotateSpeed;
  private boolean fieldOriented;
  private AHRS gyro;

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

    this.xLimiter = new SlewRateLimiter(3);
    this.yLimiter = new SlewRateLimiter(3);
    this.rotateLimiter = new SlewRateLimiter(3);

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Don't worry about it
    double xSpeed = joy.getY(); // this
    double ySpeed = joy.getX(); // this
    double rotateSpeed = joy.getTwist();

    xSpeed = MathUtil.applyDeadband(xSpeed, 0.15);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.15);
    rotateSpeed = MathUtil.applyDeadband(rotateSpeed, 0.15);

    xSpeed = xLimiter.calculate(xSpeed) * (DriveConstants.MAX_DRIVE_SPEED /*/ 4*/);
    ySpeed = yLimiter.calculate(ySpeed) * (DriveConstants.MAX_DRIVE_SPEED /*/ 4*/);
    rotateSpeed = rotateLimiter.calculate(rotateSpeed) * (DriveConstants.MAX_ROTATE_SPEED) /*/ / 4 /* */;

    if(swerveDrive.getFieldOriented()) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotateSpeed, swerveDrive.getGyro().getRotation2d());      
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed);
    }

    moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveDrive.setModuleStates(moduleStates);

    SmartDashboard.putNumber("Chassis x-speed", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis y-speed", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis rotate-speed", chassisSpeeds.omegaRadiansPerSecond);
    SmartDashboard.putNumber("Joystick X", joy.getX());
    SmartDashboard.putNumber("Joystick Y", joy.getY());
    SmartDashboard.putNumber("Joystick Z", joy.getTwist());

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
